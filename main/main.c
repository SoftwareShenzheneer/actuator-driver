#include "esp_log.h"
#include "nvs_flash.h"

#include "bluetooth_service.h"
#include "dualsense_service.h"
#include "dualsense_esp32_btc.h"

static const char *TAG = "main";

static dualsense_service_t *s_service;

static void on_report(const dualsense_state_t *state, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "LX=%3u LY=%3u RX=%3u RY=%3u | L2=%3u R2=%3u | btns=0x%08lx | batt=%u%%",
             state->left_stick_x, state->left_stick_y,
             state->right_stick_x, state->right_stick_y,
             state->trigger_l2, state->trigger_r2,
             (unsigned long)state->buttons, state->battery_level);
}

static void on_conn(dualsense_conn_state_t state, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "connection state: %d", (int)state);

    if (state == DUALSENSE_CONN_CONNECTED && s_service != NULL) {
        /* Demo: RED lightbar + a rumble burst. Red (not the default blue) so we
         * can tell whether our output report is actually taking effect. The
         * rumble stays on until reset — proper control comes later. */
        dualsense_output_t out = {
            .rumble_right = 128,
            .rumble_left  = 128,
            .led_red      = 255,
            .led_green    = 0,
            .led_blue     = 0,
        };
        dualsense_service_set_output(s_service, &out);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(bluetooth_service_init(BLUETOOTH_MODE_BTDM));

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

    if (dualsense_service_start(s_service) != 0) {
        ESP_LOGE(TAG, "failed to start dualsense service");
        return;
    }

    ESP_LOGI(TAG, "DualSense service started; put the controller in pairing mode");
}
