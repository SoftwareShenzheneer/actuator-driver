/*
 * AS5047P rotary encoder ESP32 platform backend.
 *
 * Implements rotary_encoder_transport_t on top of services/gpio (shared SPI
 * bus + manually-driven CS). The portable core does not see any ESP-IDF types;
 * this file is the adapter between the two.
 */
#pragma once

#include "rotary_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_spi_device;  /* services/gpio (opaque) */

/*
 * Fill `transport` with an implementation that exchanges 16-bit frames through
 * the ESP32 SPI device `dev` (added with gpio_spi_device_add()). `dev` must
 * outlive the encoder.
 */
void rotary_encoder_esp32_transport_init(struct gpio_spi_device *dev,
                                         rotary_encoder_transport_t *transport);

#ifdef __cplusplus
}
#endif
