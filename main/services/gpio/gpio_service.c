/*
 * GPIO/SPI platform service (ESP32 backend).
 *
 * Shared SPI bus bring-up plus a manually-CS device whose 16-bit frames are
 * exchanged with explicit byte order (high byte first, since SPI is MSB-first)
 * and software CS timing.
 */
#include "gpio_service.h"

#include <stddef.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

/* The AS5047P needs >= 350 ns from CSn falling to the first clock edge and
 * >= 350 ns CS-high between frames; 1 us gives comfortable margin. */
#define CS_DELAY_US 1

struct gpio_spi_device {
    spi_device_handle_t handle;
    int cs_io;
};

static bool s_bus_initialized;

esp_err_t gpio_spi_bus_init(spi_host_device_t host, int sclk_io, int mosi_io, int miso_io)
{
    if (s_bus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_bus_config_t bus = {
        .mosi_io_num = mosi_io,
        .miso_io_num = miso_io,
        .sclk_io_num = sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_DISABLED);
    if (err == ESP_OK) {
        s_bus_initialized = true;
    }
    return err;
}

esp_err_t gpio_spi_device_add(spi_host_device_t host, int cs_io, int clock_hz, int mode,
                              gpio_spi_device_t **out_dev)
{
    if (out_dev == NULL || cs_io < 0 || clock_hz <= 0 || mode < 0 || mode > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_dev = NULL;

    gpio_spi_device_t *dev = (gpio_spi_device_t *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->cs_io = cs_io;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cs_io,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        goto fail;
    }
    gpio_set_level(cs_io, 1);

    spi_device_interface_config_t dc = {
        .clock_speed_hz = clock_hz,
        .mode = mode,
        .spics_io_num = -1,   /* CS is driven manually in transact16() */
        .queue_size = 1,
    };
    err = spi_bus_add_device(host, &dc, &dev->handle);
    if (err != ESP_OK) {
        goto fail;
    }

    *out_dev = dev;
    return ESP_OK;

fail:
    free(dev);
    return err;
}

esp_err_t gpio_spi_device_transact16(gpio_spi_device_t *dev, uint16_t tx_word,
                                     uint16_t *rx_word)
{
    if (dev == NULL || rx_word == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Explicit bytes, high byte first, avoids the tx_buffer endianness
     * ambiguity of a multi-byte word. */
    uint8_t tx[2] = { (uint8_t)(tx_word >> 8), (uint8_t)(tx_word & 0xFF) };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_acquire_bus(dev->handle, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(dev->cs_io, 0);
    esp_rom_delay_us(CS_DELAY_US);
    err = spi_device_polling_transmit(dev->handle, &t);
    esp_rom_delay_us(CS_DELAY_US);
    gpio_set_level(dev->cs_io, 1);
    esp_rom_delay_us(CS_DELAY_US);

    spi_device_release_bus(dev->handle);

    if (err == ESP_OK) {
        *rx_word = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    }
    return err;
}

esp_err_t gpio_spi_device_remove(gpio_spi_device_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = spi_bus_remove_device(dev->handle);
    gpio_reset_pin(dev->cs_io);
    free(dev);
    return err;
}
