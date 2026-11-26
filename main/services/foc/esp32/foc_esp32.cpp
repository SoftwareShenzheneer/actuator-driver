/*
 * FOC backend for ESP32 using esp_simplefoc (the IDF-native SimpleFOC port).
 *
 * Two BLDCMotor objects in velocity mode, each with a BLDCDriver3PWM and an
 * angle sensor, driven by a ~1 kHz control task. This is a platform backend,
 * so it may include ESP-IDF headers.
 *
 * Sensor choice: flip the FOC_SENSOR_* define below (single switch).
 *   FOC_SENSOR_AS5600   onboard I2C AS5600 (12-bit)   — I2C0 18/19, I2C1 5/23
 *   FOC_SENSOR_AS5047P  external SPI AS5047P (14-bit) — SCK 18, MISO 19, MOSI 23,
 *                                                       CS_right 13, CS_left 5
 *
 * Shared motor enable: GPIO 12 (GPIO 22 also asserted for safety).
 * Right (R2) = M0: PWM 32/33/25.  Left (L2) = M1: PWM 26/27/14.
 */
#include "foc_esp32.h"

#include "esp_simplefoc.h"

#include "local_lowside_cs.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

/* =============================================================================
 * FOC angle sensor selection — switch encoders with this single define.
 *
 *   FOC_SENSOR_AS5600   (default)  onboard I2C AS5600, 12-bit
 *   FOC_SENSOR_AS5047P             external SPI AS5047P, 14-bit
 * ========================================================================== */
// #define FOC_SENSOR_AS5600
#define FOC_SENSOR_AS5047P

#if defined(FOC_SENSOR_AS5047P)
#include "as5047p_sensor.h"
#define FOC_SENSOR_RIGHT_LABEL "AS5047P SPI CS13"
#define FOC_SENSOR_LEFT_LABEL  "AS5047P SPI CS5"
#else
#define FOC_SENSOR_RIGHT_LABEL "AS5600 I2C0 18/19"
#define FOC_SENSOR_LEFT_LABEL  "AS5600 I2C1 5/23"
#endif

static const char *TAG = "foc_esp32";

#define FOC_POLE_PAIRS      7
#define FOC_VBUS            12.0f
#define FOC_ENABLE_PIN      12
#define FOC_ENABLE_PIN_ALT  22

/* Left channel (M1) has no motor/encoder wired on this build. Disable it so its
 * unplugged CS5 encoder is not polled every control tick (halves the SPI load)
 * and its velocity reads 0 instead of garbage. */
#define FOC_ENABLE_LEFT     0

/* Velocity-loop tuning. Back to the conservative baseline: aggressive gains plus
 * a short LPF made the loop amplify the 14-bit encoder's quantization noise into
 * high-frequency oscillation (rotor buzzing at rest). D stays 0. */
#define FOC_VEL_PID_P       0.2f
#define FOC_VEL_PID_I       0.3f
#define FOC_VEL_PID_D       0.0f
#define FOC_VEL_PID_RAMP    1000.0f
#define FOC_VEL_LPF_TF      0.05f
#define FOC_ALIGN_VOLTAGE   2.0f

/* Velocity setpoint slew rate (rev/s per second). The trigger sets a target; the
 * setpoint ramps toward it at this rate so the velocity PID tracks a moving
 * reference instead of chasing a step — smooths speed transitions. */
#define FOC_VEL_SLEW_REV_PER_S_PER_S  60.0f
#define FOC_VEL_SLEW_RAD_S2           (FOC_VEL_SLEW_REV_PER_S_PER_S * _2PI)

/* Control-loop period. 200 us -> 5 kHz: fast enough for clean commutation, but
 * long enough that loopFOC() (sensor + current-sense reads) fits without
 * starving the diag/BT tasks. */
#define FOC_LOOP_PERIOD_US  200

/* Right motor = M0. */
static BLDCMotor motor_right = BLDCMotor(FOC_POLE_PAIRS);
static BLDCDriver3PWM driver_right = BLDCDriver3PWM(32, 33, 25, FOC_ENABLE_PIN);

/* Left motor = M1. */
static BLDCMotor motor_left = BLDCMotor(FOC_POLE_PAIRS);
static BLDCDriver3PWM driver_left = BLDCDriver3PWM(26, 27, 14, FOC_ENABLE_PIN);

#if defined(FOC_SENSOR_AS5047P)
/* Two AS5047P encoders share one SPI bus (two CS lines). */
#define FOC_SPI_HOST  SPI2_HOST
#define FOC_SPI_SCK   GPIO_NUM_18
#define FOC_SPI_MISO  GPIO_NUM_19
#define FOC_SPI_MOSI  GPIO_NUM_23
#define FOC_SPI_CS_R  GPIO_NUM_13
#define FOC_SPI_CS_L  GPIO_NUM_5

static AS5047P sensor_right = AS5047P(FOC_SPI_HOST, FOC_SPI_SCK, FOC_SPI_MISO,
                                      FOC_SPI_MOSI, FOC_SPI_CS_R);
static AS5047P sensor_left  = AS5047P(FOC_SPI_HOST, FOC_SPI_SCK, FOC_SPI_MISO,
                                      FOC_SPI_MOSI, FOC_SPI_CS_L);
#else
static AS5600 sensor_right = AS5600(I2C_NUM_0, GPIO_NUM_18, GPIO_NUM_19);
static AS5600 sensor_left  = AS5600(I2C_NUM_1, GPIO_NUM_5, GPIO_NUM_23);
#endif

/* Right motor (M0 / 3508) current sense: 10 mOhm shunt, 50x gain, CS1/CS2 on ADC1.
 * Local low-side sense (comparator-sampled) + foc_current control. */
static LocalLowSideCurrentSense current_sense_right = LocalLowSideCurrentSense(0.01f, 50.0f, 34, 35);

typedef struct {
    bool present[FOC_MOTOR_COUNT];
    volatile float target[FOC_MOTOR_COUNT];   /* requested velocity (from trigger) */
    volatile float setpoint[FOC_MOTOR_COUNT]; /* ramped velocity the PID tracks */
} foc_esp32_ctx_t;

static foc_esp32_ctx_t s_ctx = {
    .present  = { false, false },
    .target   = { 0.0f, 0.0f },
    .setpoint = { 0.0f, 0.0f },
};

/* The V3P board has a single motor-enable line (GPIO 12). Drive it directly and
 * keep it high; GPIO 22 is also asserted for safety on some revisions. */
static void assert_enable(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << FOC_ENABLE_PIN) | (1ULL << FOC_ENABLE_PIN_ALT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level((gpio_num_t)FOC_ENABLE_PIN, 1);
    gpio_set_level((gpio_num_t)FOC_ENABLE_PIN_ALT, 1);
}

template <typename SensorT>
static void setup_motor(BLDCMotor &motor, BLDCDriver3PWM &driver, SensorT &sensor,
                        int mcpwm_group)
{
    sensor.init();
    sensor.min_elapsed_time = 0.002f; /* 2 ms velocity window: smooths quantization without 10ms jolt */
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = FOC_VBUS;
    driver.voltage_limit        = FOC_VBUS;
#if CONFIG_SOC_MCPWM_SUPPORTED
    driver.init(mcpwm_group);
#else
    driver.init({ 1, 2, 3 });
#endif

    motor.linkDriver(&driver);
    motor.controller               = MotionControlType::velocity;
    motor.PID_velocity.P           = FOC_VEL_PID_P;
    motor.PID_velocity.I           = FOC_VEL_PID_I;
    motor.PID_velocity.D           = FOC_VEL_PID_D;
    motor.PID_velocity.output_ramp = FOC_VEL_PID_RAMP;
    motor.LPF_velocity.Tf          = FOC_VEL_LPF_TF;
    motor.voltage_limit            = FOC_VBUS;
    motor.velocity_limit           = FOC_MAX_VELOCITY_RAD_S;
    motor.voltage_sensor_align     = FOC_ALIGN_VOLTAGE;
    motor.init();
}

/* ----------------------------------------------------------------------------
 * Temporary diagnostic: map PWM driver channels to angle sensors.
 *
 * The V3P wiring can be cross-connected (phase connectors swapped between the
 * two motors, or one sensor not yet wired). These helpers print the raw sensor
 * angles and energise each driver channel open-loop so we can see, from the
 * log, which sensor is physically co-located with which driver.
 * Remove this block once the mapping is confirmed.
 * -------------------------------------------------------------------------- */
#define DIAG_RAD_TO_DEG (180.0f / _PI)

static void diag_log_raw_angles(void)
{
    float s0 = sensor_right.getSensorAngle();
    float s1 = sensor_left.getSensorAngle();
    ESP_LOGI(TAG, "diag angles: S0(%s)=%.1f deg  S1(%s)=%.1f deg",
             FOC_SENSOR_RIGHT_LABEL, s0 * DIAG_RAD_TO_DEG,
             FOC_SENSOR_LEFT_LABEL,  s1 * DIAG_RAD_TO_DEG);
    ESP_LOGI(TAG, "diag vel: R=%.0f RPM (tgt %.0f)  L=%.0f RPM",
             motor_right.shaft_velocity * 60.0f / _2PI,
             s_ctx.target[FOC_MOTOR_RIGHT] * 60.0f / _2PI,
             motor_left.shaft_velocity * 60.0f / _2PI);
    ESP_LOGI(TAG, "diag I: iq=%.2f A  id=%.2f A", motor_right.current.q, motor_right.current.d);
    sensor_right.dumpDiagnostics("R");
    sensor_left.dumpDiagnostics("L");
}

static void diag_log_angle_burst(void)
{
    const int N = 50;
    float prev = sensor_right.getSensorAngle();
    char line[512] = {0};
    int pos = 0;
    for (int i = 0; i < N; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
        float cur = sensor_right.getSensorAngle();
        float d = (cur - prev) * DIAG_RAD_TO_DEG;
        if (d > 180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        prev = cur;
        int w = snprintf(line + pos, sizeof(line) - pos, "%.1f,", d);
        if (w < 0 || pos + w >= (int)sizeof(line) - 8) break;
        pos += w;
    }
    ESP_LOGI(TAG, "burst: %s", line);
}

static void diag_task(void *arg)
{
    (void)arg;
    while (1) {
        diag_log_raw_angles();
        diag_log_angle_burst();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void diag_probe_channel(const char *label, BLDCMotor &motor)
{
    float s0_before = sensor_right.getSensorAngle();
    float s1_before = sensor_left.getSensorAngle();

    ESP_LOGI(TAG, "diag PROBE %s: energising ~1s (watch which motor moves)", label);

    /* Sweep one electrical revolution so the rotor steps clearly (~51 deg mech). */
    for (int i = 0; i <= 250; i++) {
        float angle_el = _3PI_2 + _2PI * (float)i / 250.0f;
        motor.setPhaseVoltage(FOC_ALIGN_VOLTAGE, 0.0f, angle_el);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    {
        PhaseCurrent_s pc = current_sense_right.getPhaseCurrents();
        ESP_LOGI(TAG, "diag I probe(%s): Ia=%.2f Ib=%.2f Ic=%.2f", label, pc.a, pc.b, pc.c);
    }
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);

    float s0_after = sensor_right.getSensorAngle();
    float s1_after = sensor_left.getSensorAngle();

    float d0 = fabsf(s0_after - s0_before);
    float d1 = fabsf(s1_after - s1_before);
    if (d0 > _PI) d0 = _2PI - d0;
    if (d1 > _PI) d1 = _2PI - d1;

    ESP_LOGI(TAG, "diag PROBE %s done: S0 moved %.1f deg, S1 moved %.1f deg",
             label, d0 * DIAG_RAD_TO_DEG, d1 * DIAG_RAD_TO_DEG);
}

static TaskHandle_t s_control_task = NULL;
static esp_timer_handle_t s_foc_timer = NULL;

/* FOC_LOOP_PERIOD_US periodic ISR: kick the control task once per period. */
static void foc_timer_cb(void *arg)
{
    (void)arg;
    xTaskNotifyGive(s_control_task);
}

static void control_task(void *arg)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)arg;
    const float slew_step = FOC_VEL_SLEW_RAD_S2 * (FOC_LOOP_PERIOD_US / 1000000.0f);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Ramp each setpoint toward its requested target before commanding the
         * velocity loop, so the PID tracks a smooth reference instead of a step. */
        for (int m = 0; m < FOC_MOTOR_COUNT; m++) {
            if (!c->present[m]) {
                continue;
            }
            float err = c->target[m] - c->setpoint[m];
            if (err > slew_step) {
                c->setpoint[m] += slew_step;
            } else if (err < -slew_step) {
                c->setpoint[m] -= slew_step;
            } else {
                c->setpoint[m] = c->target[m];
            }
        }

        if (c->present[FOC_MOTOR_RIGHT]) {
            motor_right.loopFOC();
            motor_right.move(c->setpoint[FOC_MOTOR_RIGHT]);
        }
        if (c->present[FOC_MOTOR_LEFT]) {
            motor_left.loopFOC();
            motor_left.move(c->setpoint[FOC_MOTOR_LEFT]);
        }
    }
}

static int foc_esp32_init(void *ctx)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)ctx;

    setup_motor(motor_right, driver_right, sensor_right, 0);
    setup_motor(motor_left,  driver_left,  sensor_left,  1);

    c->present[FOC_MOTOR_RIGHT] = false;
    c->present[FOC_MOTOR_LEFT]  = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (!c->present[FOC_MOTOR_RIGHT])
            c->present[FOC_MOTOR_RIGHT] = (motor_right.initFOC() != 0);
#if FOC_ENABLE_LEFT
        if (!c->present[FOC_MOTOR_LEFT])
            c->present[FOC_MOTOR_LEFT]  = (motor_left.initFOC() != 0);
#endif
        if (c->present[FOC_MOTOR_RIGHT])
            break;
    }
    assert_enable();   /* a failed align may have driven the shared enable low */

    /* Monitoring-only current sense, linked after alignment (voltage control). */
    current_sense_right.linkDriver(&driver_right);
    current_sense_right.init();
    motor_right.linkCurrentSense(&current_sense_right);

    if (!c->present[FOC_MOTOR_RIGHT])
        ESP_LOGE(TAG, "RIGHT motor not detected (PWM 32/33/25, %s).", FOC_SENSOR_RIGHT_LABEL);
#if FOC_ENABLE_LEFT
    if (!c->present[FOC_MOTOR_LEFT])
        ESP_LOGE(TAG, "LEFT motor not detected (PWM 26/27/14, %s).", FOC_SENSOR_LEFT_LABEL);
    if (c->present[FOC_MOTOR_RIGHT] && c->present[FOC_MOTOR_LEFT])
        ESP_LOGI(TAG, "Both motors detected and aligned.");
    else if (!c->present[FOC_MOTOR_RIGHT] && !c->present[FOC_MOTOR_LEFT])
        ESP_LOGE(TAG, "No motors detected - check the 12 V supply and phase wiring.");
#else
    if (c->present[FOC_MOTOR_RIGHT])
        ESP_LOGI(TAG, "Right motor detected and aligned (left channel disabled).");
#endif

    /* Temporary diagnostic: probe driver→sensor mapping before the control task
     * starts (so the velocity loop can't fight the probe sweep). */
    diag_log_raw_angles();
    diag_probe_channel("RIGHT M0 (32/33/25)", motor_right);
#if FOC_ENABLE_LEFT
    vTaskDelay(pdMS_TO_TICKS(300));
    diag_probe_channel("LEFT  M1 (26/27/14)", motor_left);
#endif

    xTaskCreatePinnedToCore(control_task, "foc_control", 4096, c, 5, &s_control_task, 1);
    xTaskCreatePinnedToCore(diag_task, "foc_diag", 4096, NULL, 3, NULL, 1);

    esp_timer_create_args_t timer_args = {
        .callback               = foc_timer_cb,
        .arg                    = NULL,
        .dispatch_method        = ESP_TIMER_TASK,
        .name                   = "foc_loop",
        .skip_unhandled_events  = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_foc_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_foc_timer, FOC_LOOP_PERIOD_US));
    return 0;
}

static int foc_esp32_present(void *ctx, foc_motor_t motor)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)ctx;
    if (motor >= FOC_MOTOR_COUNT) {
        return 0;
    }
    return c->present[motor] ? 1 : 0;
}

static void foc_esp32_set_velocity(void *ctx, foc_motor_t motor, float rad_s)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)ctx;
    if (motor >= FOC_MOTOR_COUNT) {
        return;
    }
    c->target[motor] = rad_s;
}

static float foc_esp32_get_velocity(void *ctx, foc_motor_t motor)
{
    (void)ctx;
    switch (motor) {
    case FOC_MOTOR_RIGHT: return motor_right.shaft_velocity;
    case FOC_MOTOR_LEFT:  return motor_left.shaft_velocity;
    default:              return 0.0f;
    }
}

static const foc_ops_t s_ops = {
    .init         = foc_esp32_init,
    .present      = foc_esp32_present,
    .set_velocity = foc_esp32_set_velocity,
    .get_velocity = foc_esp32_get_velocity,
};

extern "C" foc_t *foc_esp32_create(void)
{
    return foc_create(&s_ops, &s_ctx);
}
