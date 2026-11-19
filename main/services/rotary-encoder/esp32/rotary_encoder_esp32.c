/*
 * AS5047P rotary encoder ESP32 platform backend.
 *
 * The single-frame exchange in the portable core is backed by
 * gpio_spi_device_transact16() from services/gpio, which owns the bus and the
 * manual CS timing.
 */
#include "rotary_encoder_esp32.h"

#include "esp_err.h"
#include "gpio_service.h"

static int esp32_xfer(void *ctx, uint16_t tx_word, uint16_t *rx_word)
{
    esp_err_t err = gpio_spi_device_transact16((gpio_spi_device_t *)ctx, tx_word, rx_word);
    return (err == ESP_OK) ? 0 : -1;
}

void rotary_encoder_esp32_transport_init(struct gpio_spi_device *dev,
                                         rotary_encoder_transport_t *transport)
{
    if (dev == NULL || transport == NULL) {
        return;
    }

    transport->xfer = esp32_xfer;
    transport->ctx = dev;
}
