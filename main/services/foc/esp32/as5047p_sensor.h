/*
 * AS5047P magnetic rotary encoder — SimpleFOC Sensor backend for the ESP32.
 *
 * Mirrors the esp_simplefoc port's AS5048A driver: a 14-bit AMS angle sensor on
 * SPI (mode 1), read with the IDF SPI driver. The AS5047P uses the same frame
 * format and ANGLECOM (0x3FFF) angle register as the AS5048A/AS5147.
 *
 * Two sensors can share one SPI bus: init() tolerates the bus already being
 * initialised (it only creates the bus once) and then adds this sensor's own CS.
 */
#pragma once

#include "common/base_classes/Sensor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class AS5047P : public Sensor {
public:
    AS5047P(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io,
            gpio_num_t mosi_io, gpio_num_t cs_io);
    ~AS5047P();

    void  init();
    void  deinit();
    float getSensorAngle();
    void  dumpDiagnostics(const char *label);

private:
    uint16_t       readRegister(uint16_t reg_address);
    uint16_t       readRegisterRaw(uint16_t reg_address);
    uint16_t       readRawPosition();
    static uint8_t calculateParity(uint16_t value);

    spi_host_device_t   _spi_host;
    gpio_num_t          _sclk_io;
    gpio_num_t          _miso_io;
    gpio_num_t          _mosi_io;
    gpio_num_t          _cs_io;
    spi_device_handle_t _spi_device;
    SemaphoreHandle_t   _mutex;   /* serializes readRegister across tasks */
    bool                _is_installed;
};
