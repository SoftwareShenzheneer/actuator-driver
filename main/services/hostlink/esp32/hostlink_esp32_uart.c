/*
 * Host-link UART backend for ESP32 (esp_driver_uart).
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/uart.h"

#include "hostlink_esp32_uart.h"

static const char *TAG = "hostlink_uart";

#define HOSTLINK_UART_NUM       UART_NUM_1
#define HOSTLINK_UART_TX_GPIO   CONFIG_HOSTLINK_UART_TX_GPIO
#define HOSTLINK_UART_RX_GPIO   CONFIG_HOSTLINK_UART_RX_GPIO
#define HOSTLINK_UART_BAUDRATE  CONFIG_HOSTLINK_UART_BAUDRATE

#define HOSTLINK_PINGPONG_INTERVAL_MS  1000
#define HOSTLINK_PINGPONG_TIMEOUT_MS    500

esp_err_t hostlink_esp32_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = HOSTLINK_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(HOSTLINK_UART_NUM, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        return err;
    }

    err = uart_param_config(HOSTLINK_UART_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        return err;
    }

    err = uart_set_pin(HOSTLINK_UART_NUM, HOSTLINK_UART_TX_GPIO, HOSTLINK_UART_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "host link UART%d @ %d baud (TX=%d, RX=%d)",
             HOSTLINK_UART_NUM, HOSTLINK_UART_BAUDRATE,
             HOSTLINK_UART_TX_GPIO, HOSTLINK_UART_RX_GPIO);

    return ESP_OK;
}

esp_err_t hostlink_esp32_uart_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(HOSTLINK_UART_NUM, data, len);
    return (written < 0) ? ESP_FAIL : ESP_OK;
}

static void pingpong_task(void *arg)
{
    (void)arg;

    static const char *ping = "ping\n";
    uint8_t buf[64];

    while (1) {
        hostlink_esp32_uart_send((const uint8_t *)ping, 5);

        bool got_pong = false;
        size_t pos = 0;
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(HOSTLINK_PINGPONG_TIMEOUT_MS);

        /* Collect whatever comes back before the deadline and look for "pong". */
        while (xTaskGetTickCount() < deadline && pos < sizeof(buf) - 1) {
            int n = uart_read_bytes(HOSTLINK_UART_NUM, buf + pos, sizeof(buf) - 1 - pos,
                                    pdMS_TO_TICKS(20));
            if (n > 0) {
                pos += (size_t)n;
                buf[pos] = 0;
                if (strstr((const char *)buf, "pong") != NULL) {
                    got_pong = true;
                    break;
                }
            }
        }

        if (got_pong) {
            ESP_LOGI(TAG, "ping -> pong (connection ok)");
        } else {
            ESP_LOGW(TAG, "ping sent, no pong");
        }

        vTaskDelay(pdMS_TO_TICKS(HOSTLINK_PINGPONG_INTERVAL_MS));
    }
}

esp_err_t hostlink_esp32_uart_start_pingpong(void)
{
    if (xTaskCreate(pingpong_task, "hostlink_pingpong", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
