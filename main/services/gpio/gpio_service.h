/*
 * GPIO/SPI platform service (ESP32 backend).
 *
 * Owns the shared SPI bus and provides a small device abstraction for a
 * peripheral whose chip-select must be driven manually (bit-banged). The
 * AS5047P rotary encoder needs >= 350 ns of CS setup/hold around each frame,
 * which the ESP32 SPI peripheral's hardware CS cannot provide, so CS is toggled
 * from software with ~1 us margins.
 *
 * This is a platform service (like services/bluetooth): it may include ESP-IDF
 * headers and is not portable. Higher-level portable code (the rotary encoder
 * core) talks to the SPI bus through services/rotary-encoder/esp32/, not here.
 */
#pragma once

#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle for a manually-CS SPI device on the shared bus. */
typedef struct gpio_spi_device gpio_spi_device_t;

/* Initialize the shared SPI bus once. sclk/mosi/miso are GPIO numbers. */
esp_err_t gpio_spi_bus_init(spi_host_device_t host, int sclk_io, int mosi_io, int miso_io);

/*
 * Add a device to the bus with a manually-driven CS.
 *
 *   host     : SPI host (SPI2_HOST / SPI3_HOST); its bus must already be up.
 *   cs_io    : CS GPIO, configured as push-pull output, idle high.
 *   clock_hz : device clock in Hz (> 0).
 *   mode     : SPI mode 0..3 (CPOL/CPHA).
 *
 * The device is added with spics_io_num = -1; CS is driven by
 * gpio_spi_device_transact16().
 */
esp_err_t gpio_spi_device_add(spi_host_device_t host, int cs_io, int clock_hz, int mode,
                              gpio_spi_device_t **out_dev);

/*
 * Exchange one 16-bit frame (MSB-first) with the device.
 *
 * Drives CS low, waits ~1 us, shifts 16 bits, waits ~1 us, drives CS high, then
 * waits ~1 us. The bus is acquired for the whole CS window so another device on
 * the same bus cannot drive SCK/MISO while this device's CS is asserted.
 */
esp_err_t gpio_spi_device_transact16(gpio_spi_device_t *dev, uint16_t tx_word,
                                     uint16_t *rx_word);

/* Remove the device, reset its CS GPIO, and free the handle. */
esp_err_t gpio_spi_device_remove(gpio_spi_device_t *dev);

#ifdef __cplusplus
}
#endif
