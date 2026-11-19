#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "bluetooth_service.h"
#include "gamepad.h"
#include "host_protocol.h"
#include "hostlink_esp32_uart.h"

#if defined(CONFIG_GAMEPAD_DUALSENSE)
#include "dualsense_service.h"
#include "dualsense_esp32_btc.h"
#define GAMEPAD_NAME "DualSense"
#elif defined(CONFIG_GAMEPAD_DUALSHOCK4)
#include "dualshock4_service.h"
#include "dualshock4_esp32_btc.h"
#define GAMEPAD_NAME "DualShock 4"
#else
#error "No gamepad model selected (CONFIG_GAMEPAD_DUALSENSE / CONFIG_GAMEPAD_DUALSHOCK4)"
#endif

static const char *TAG = "main";

/* Serialize the connection status + motor value and push it to the host. */
static void send_status(gamepad_conn_state_t conn, uint8_t motor_value)
{
    host_status_payload_t payload = {
        .conn        = (uint8_t)conn,
        .motor_value = motor_value,
    };
    uint8_t msg[HOST_PROTOCOL_STATUS_FRAME_LEN];
    size_t len = host_protocol_build_status(msg, &payload);
    if (len > 0) {
        hostlink_esp32_uart_send(msg, len);
    }
}

#if defined(CONFIG_GAMEPAD_DUALSENSE)

static dualsense_service_t *s_service;
static dualsense_output_t s_output;      /* last output; re-sent as keepalive */
static TimerHandle_t s_keepalive_timer;  /* periodic output to keep the DualSense link alive */
static gamepad_conn_state_t s_conn = GAMEPAD_CONN_DISCONNECTED;
static uint8_t s_motor = 0;

/* A genuine DualSense drops the Bluetooth link after ~20 s of host silence, so
 * re-send the output report periodically to keep the connection alive. */
#define DUALSENSE_KEEPALIVE_INTERVAL_MS 2000
#define DUALSENSE_LOG_INTERVAL_MS 5000

static void keepalive_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (s_service != NULL) {
        dualsense_service_set_output(s_service, &s_output);
    }
}

static void on_report(const dualsense_state_t *state, void *user)
{
    (void)user;

    /* Push the latest analog-derived motor value to the host on every report. */
    gamepad_analog_t analog;
    gamepad_analog_from_dualsense(state, &analog);
    s_motor = gamepad_motor_value(&analog);
    send_status(s_conn, s_motor);

    /* Throttle: full 0x31 reports arrive at high rate; logging every one floods
     * the UART. Print a snapshot once every DUALSENSE_LOG_INTERVAL_MS. */
    static TickType_t last_log = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_log) < pdMS_TO_TICKS(DUALSENSE_LOG_INTERVAL_MS)) {
        return;
    }
    last_log = now;

    ESP_LOGI(TAG, "LX=%3u LY=%3u RX=%3u RY=%3u | L2=%3u R2=%3u | btns=0x%08lx | batt=%u%%",
             state->left_stick_x, state->left_stick_y,
             state->right_stick_x, state->right_stick_y,
             state->trigger_l2, state->trigger_r2,
             (unsigned long)state->buttons, state->battery_level);
}

static void on_conn(dualsense_conn_state_t state, void *user)
{
    (void)user;

    s_conn = gamepad_conn_from_dualsense(state);
    if (state != DUALSENSE_CONN_CONNECTED) {
        s_motor = 0;
    }
    send_status(s_conn, s_motor);

    ESP_LOGI(TAG, "connection state: %d", (int)state);

    if (state == DUALSENSE_CONN_CONNECTED && s_service != NULL) {
        /* Demo: RED lightbar + a rumble burst. Red (not the default blue) so we
         * can tell whether our output report is actually taking effect. The
         * rumble stays on until reset — proper control comes later. */
        s_output = (dualsense_output_t) {
            .rumble_right = 128,
            .rumble_left  = 128,
            .led_red      = 255,
            .led_green    = 0,
            .led_blue     = 0,
        };
        dualsense_service_set_output(s_service, &s_output);

        if (s_keepalive_timer != NULL) {
            xTimerStart(s_keepalive_timer, 0);
        }
    } else if ((state == DUALSENSE_CONN_DISCONNECTED || state == DUALSENSE_CONN_ERROR) &&
               s_keepalive_timer != NULL) {
        xTimerStop(s_keepalive_timer, 0);
    }
}

static void start_service(void)
{
    dualsense_transport_t *transport = dualsense_esp32_btc_get_default();

    dualsense_callbacks_t cbs = {
        .report_cb = on_report,
        .conn_cb   = on_conn,
        .user      = NULL,
    };

    s_service = dualsense_service_create(transport, &cbs);
    if (s_service == NULL) {
        ESP_LOGE(TAG, "failed to create dualsense service");
        return;
    }

    s_keepalive_timer = xTimerCreate("dualsense_keepalive",
                                     pdMS_TO_TICKS(DUALSENSE_KEEPALIVE_INTERVAL_MS),
                                     pdTRUE, NULL, keepalive_timer_cb);

    if (dualsense_service_start(s_service) != 0) {
        ESP_LOGE(TAG, "failed to start dualsense service");
        return;
    }
}

#elif defined(CONFIG_GAMEPAD_DUALSHOCK4)

static dualshock4_service_t *s_service;
static dualshock4_output_t s_output;      /* last output; re-sent as keepalive */
static TimerHandle_t s_keepalive_timer;   /* periodic output to keep the DS4 link alive */
static gamepad_conn_state_t s_conn = GAMEPAD_CONN_DISCONNECTED;
static uint8_t s_motor = 0;

/* A genuine DS4 drops the Bluetooth link after ~20 s of host silence, so re-send
 * the output report periodically to keep the connection alive. */
#define DS4_KEEPALIVE_INTERVAL_MS 2000
#define DS4_LOG_INTERVAL_MS 5000

static void keepalive_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (s_service != NULL) {
        dualshock4_service_set_output(s_service, &s_output);
    }
}

static void on_report(const dualshock4_state_t *state, void *user)
{
    (void)user;

    /* Push the latest analog-derived motor value to the host on every report. */
    gamepad_analog_t analog;
    gamepad_analog_from_dualshock4(state, &analog);
    s_motor = gamepad_motor_value(&analog);
    send_status(s_conn, s_motor);

    /* Throttle: full 0x11 reports arrive at 250 Hz; logging every one floods the
     * UART. Print a snapshot once every DS4_LOG_INTERVAL_MS. */
    static TickType_t last_log = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_log) < pdMS_TO_TICKS(DS4_LOG_INTERVAL_MS)) {
        return;
    }
    last_log = now;

    if (state->battery_level == 0xFF) {
        ESP_LOGI(TAG, "LX=%3u LY=%3u RX=%3u RY=%3u | L2=%3u R2=%3u | btns=0x%08lx | batt=--",
                 state->left_stick_x, state->left_stick_y,
                 state->right_stick_x, state->right_stick_y,
                 state->trigger_l2, state->trigger_r2,
                 (unsigned long)state->buttons);
    } else {
        ESP_LOGI(TAG, "LX=%3u LY=%3u RX=%3u RY=%3u | L2=%3u R2=%3u | btns=0x%08lx | batt=%u%%",
                 state->left_stick_x, state->left_stick_y,
                 state->right_stick_x, state->right_stick_y,
                 state->trigger_l2, state->trigger_r2,
                 (unsigned long)state->buttons, state->battery_level);
    }
}

static void on_conn(dualshock4_conn_state_t state, void *user)
{
    (void)user;

    s_conn = gamepad_conn_from_dualshock4(state);
    if (state != DUALSHOCK4_CONN_CONNECTED) {
        s_motor = 0;
    }
    send_status(s_conn, s_motor);

    ESP_LOGI(TAG, "connection state: %d", (int)state);

    if (state == DUALSHOCK4_CONN_CONNECTED && s_service != NULL) {
        /* Demo: RED lightbar + a rumble burst. Red (not the default blue) so we
         * can tell whether our output report is actually taking effect. The
         * rumble stays on until reset — proper control comes later. */
        s_output = (dualshock4_output_t) {
            .motor_left  = 128,   /* strong rumble: whole body */
            .motor_right = 128,   /* weak rumble: grips */
            .led_red     = 255,
            .led_green   = 0,
            .led_blue    = 0,
        };
        dualshock4_service_set_output(s_service, &s_output);

        if (s_keepalive_timer != NULL) {
            xTimerStart(s_keepalive_timer, 0);
        }
    } else if ((state == DUALSHOCK4_CONN_DISCONNECTED || state == DUALSHOCK4_CONN_ERROR) &&
               s_keepalive_timer != NULL) {
        xTimerStop(s_keepalive_timer, 0);
    }
}

static void start_service(void)
{
    dualshock4_transport_t *transport = dualshock4_esp32_btc_get_default();

    dualshock4_callbacks_t cbs = {
        .report_cb = on_report,
        .conn_cb   = on_conn,
        .user      = NULL,
    };

    s_service = dualshock4_service_create(transport, &cbs);
    if (s_service == NULL) {
        ESP_LOGE(TAG, "failed to create dualshock4 service");
        return;
    }

    s_keepalive_timer = xTimerCreate("ds4_keepalive",
                                     pdMS_TO_TICKS(DS4_KEEPALIVE_INTERVAL_MS),
                                     pdTRUE, NULL, keepalive_timer_cb);

    if (dualshock4_service_start(s_service) != 0) {
        ESP_LOGE(TAG, "failed to start dualshock4 service");
        return;
    }
}

#endif

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(bluetooth_service_init(BLUETOOTH_MODE_BTDM));
    ESP_ERROR_CHECK(hostlink_esp32_uart_init());
    ESP_ERROR_CHECK(hostlink_esp32_uart_start_pingpong());

    start_service();

    ESP_LOGI(TAG, "%s service started; put the controller in pairing mode", GAMEPAD_NAME);
}
