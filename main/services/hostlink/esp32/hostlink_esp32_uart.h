/*
 * Host-link UART backend (ESP32).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the host-link UART (default 921600 8N1, pins from Kconfig).
 * Call once after startup. */
esp_err_t hostlink_esp32_uart_init(void);

/* Send `len` bytes to the host over the link. */
esp_err_t hostlink_esp32_uart_send(const uint8_t *data, size_t len);

/* Start the ping/pong link check: send "ping" periodically and log when "pong"
 * comes back. Call once after hostlink_esp32_uart_init(). */
esp_err_t hostlink_esp32_uart_start_pingpong(void);

#ifdef __cplusplus
}
#endif
