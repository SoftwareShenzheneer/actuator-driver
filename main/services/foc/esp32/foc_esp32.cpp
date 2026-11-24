/*
 * FOC backend for ESP32 using esp_simplefoc (the IDF-native SimpleFOC port).
 *
 * Mirrors ../simplefoc/main.cpp: two BLDCMotor objects in velocity mode, each
 * with a BLDCDriver3PWM + AS5600 angle sensor, driven by a ~1 kHz control task.
 * This is a platform backend, so it may include ESP-IDF headers.
 *
 * DengFOC V3P pinout (right == M0, left == M1):
 *   right (R2) = M0: PWM 32/33/25, AS5600 on I2C0 (SCL 18, SDA 19)
 *   left  (L2) = M1: PWM 26/27/14, AS5600 on I2C1 (SCL 5,  SDA 23)
 *   shared motor enable: GPIO 12 (GPIO 22 also asserted for safety)
 */
#include "foc_esp32.h"

#include "esp_simplefoc.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "foc_esp32";

#define FOC_POLE_PAIRS      7
#define FOC_VBUS            12.0f
#define FOC_ENABLE_PIN      12
#define FOC_ENABLE_PIN_ALT  22

/* Velocity-loop tuning. Kept gentle for the gimbal motor on a 12 V rail: low P,
 * low I (to avoid integral wind-up from the noisy 12-bit AS5600 velocity), no D
 * (D amplifies sensor noise), and the default velocity LPF. */
#define FOC_VEL_PID_P       0.3f
#define FOC_VEL_PID_I       0.5f
#define FOC_VEL_PID_D       0.0f
#define FOC_VEL_PID_RAMP    1000.0f
#define FOC_VEL_LPF_TF      0.05f
#define FOC_ALIGN_VOLTAGE   3.0f

/* Right motor = M0. */
static BLDCMotor motor_right = BLDCMotor(FOC_POLE_PAIRS);
static BLDCDriver3PWM driver_right = BLDCDriver3PWM(32, 33, 25, FOC_ENABLE_PIN);
static AS5600 sensor_right = AS5600(I2C_NUM_0, GPIO_NUM_18, GPIO_NUM_19);

/* Left motor = M1. */
static BLDCMotor motor_left = BLDCMotor(FOC_POLE_PAIRS);
static BLDCDriver3PWM driver_left = BLDCDriver3PWM(26, 27, 14, FOC_ENABLE_PIN);
static AS5600 sensor_left = AS5600(I2C_NUM_1, GPIO_NUM_5, GPIO_NUM_23);

typedef struct {
    bool present[FOC_MOTOR_COUNT];
    volatile float target[FOC_MOTOR_COUNT];
} foc_esp32_ctx_t;

static foc_esp32_ctx_t s_ctx = {
    .present = { false, false },
    .target  = { 0.0f, 0.0f },
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

static void setup_motor(BLDCMotor &motor, BLDCDriver3PWM &driver, AS5600 &sensor,
                        int mcpwm_group)
{
    sensor.init();
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

static void control_task(void *arg)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)arg;

    while (1) {
        if (c->present[FOC_MOTOR_RIGHT]) {
            motor_right.loopFOC();
            motor_right.move(c->target[FOC_MOTOR_RIGHT]);
        }
        if (c->present[FOC_MOTOR_LEFT]) {
            motor_left.loopFOC();
            motor_left.move(c->target[FOC_MOTOR_LEFT]);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static int foc_esp32_init(void *ctx)
{
    foc_esp32_ctx_t *c = (foc_esp32_ctx_t *)ctx;

    setup_motor(motor_right, driver_right, sensor_right, 0);
    setup_motor(motor_left,  driver_left,  sensor_left,  1);

    c->present[FOC_MOTOR_RIGHT] = (motor_right.initFOC() != 0);
    c->present[FOC_MOTOR_LEFT]  = (motor_left.initFOC() != 0);
    assert_enable();   /* a failed align may have driven the shared enable low */

    if (!c->present[FOC_MOTOR_RIGHT])
        ESP_LOGE(TAG, "RIGHT motor not detected (PWM 32/33/25, AS5600 I2C0 18/19).");
    if (!c->present[FOC_MOTOR_LEFT])
        ESP_LOGE(TAG, "LEFT motor not detected (PWM 26/27/14, AS5600 I2C1 5/23).");
    if (c->present[FOC_MOTOR_RIGHT] && c->present[FOC_MOTOR_LEFT])
        ESP_LOGI(TAG, "Both motors detected and aligned.");
    else if (!c->present[FOC_MOTOR_RIGHT] && !c->present[FOC_MOTOR_LEFT])
        ESP_LOGE(TAG, "No motors detected - check the 12 V supply and phase wiring.");

    xTaskCreate(control_task, "foc_control", 4096, c, 5, NULL);
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
