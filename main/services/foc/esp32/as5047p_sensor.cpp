/*
 * AS5047P magnetic rotary encoder — SimpleFOC Sensor backend for the ESP32.
 *
 * See as5047p_sensor.h. Protocol is identical to the AMS AS5048A: a 16-bit
 * command frame (bit15 parity, bit14 R/W, bits13:0 address) and a 16-bit reply
 * (bit15 parity, bit14 error flag, bits13:0 data). Angle = ANGLECOM (0x3FFF).
 */
#include "as5047p_sensor.h"
#include "esp_log.h"
#include "esp_check.h"

#define AS5047P_REG_ANGLE      0x3FFF
#define AS5047P_SPI_CLOCK_HZ   (10 * 1000 * 1000)  /* AS5047P datasheet max */
#define PI                     3.14159265358979f

static const char *TAG = "AS5047P";

AS5047P::AS5047P(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io,
                 gpio_num_t mosi_io, gpio_num_t cs_io)
{
    _spi_host = spi_host;
    _sclk_io = sclk_io;
    _miso_io = miso_io;
    _mosi_io = mosi_io;
    _cs_io = cs_io;
    _is_installed = false;
    _spi_device = NULL;
    _mutex = NULL;
}

AS5047P::~AS5047P()
{
    if (_is_installed) {
        deinit();
    }
}

void AS5047P::deinit()
{
    /* Only remove this device — the bus is shared with the other AS5047P. */
    if (_spi_device != NULL) {
        spi_bus_remove_device(_spi_device);
        _spi_device = NULL;
    }
    if (_mutex != NULL) {
        vSemaphoreDelete(_mutex);
        _mutex = NULL;
    }
    _is_installed = false;
}

void AS5047P::init()
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = _mosi_io,
        .miso_io_num = _miso_io,
        .sclk_io_num = _sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };
    /* Two AS5047P sensors share one bus: only the first init() should create it.
     * A second call returns ESP_ERR_INVALID_STATE, which we accept. */
    ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init fail: %d", (int)ret);
        return;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 1,
        .cs_ena_pretrans = 1,
        .clock_speed_hz = AS5047P_SPI_CLOCK_HZ,
        .spics_io_num = _cs_io,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(_spi_host, &devcfg, &_spi_device);
    ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI bus add device fail");

    if (_mutex == NULL) {
        _mutex = xSemaphoreCreateMutex();
    }

    _is_installed = true;
    Sensor::init();
    ESP_LOGI(TAG, "AS5047P init success (CS=%d)", (int)_cs_io);
}

uint8_t AS5047P::calculateParity(uint16_t value)
{
    uint8_t count = 0;
    for (int i = 0; i < 16; i++) {
        if (value & 0x1) {
            count++;
        }
        value >>= 1;
    }
    return count & 0x1;
}

uint16_t AS5047P::readRegisterRaw(uint16_t reg_address)
{
    uint16_t command = 1 << 14; /* PAR=0, R/W=R (read command) */
    command |= reg_address;
    command |= ((uint16_t)calculateParity(command) << 15); /* even parity over bits 14:0 */

    uint16_t nop = 1 << 14; /* R/W=R read of NOP (0x0000) */
    nop |= ((uint16_t)calculateParity(nop) << 15);

    /* The command and response are two separate transfers, so a task must not be
     * preempted by another read of the same sensor between them. Serialize the
     * whole register read to keep the pipelined read atomic. */
    if (_mutex != NULL) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }

    uint8_t cmd_buf[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
    uint8_t nop_buf[2] = { (uint8_t)(nop >> 8), (uint8_t)(nop & 0xFF) };
    spi_transaction_t t = {};
    t.length = 16; /* 16 bits */
    t.tx_buffer = cmd_buf;
    t.rx_buffer = NULL;
    spi_device_transmit(_spi_device, &t);

    uint8_t rx_buffer[2] = {};
    t.length = 16; /* 16 bits */
    t.tx_buffer = nop_buf;
    t.rxlength = 16; /* 16 bits */
    t.rx_buffer = rx_buffer;
    spi_device_transmit(_spi_device, &t);

    if (_mutex != NULL) {
        xSemaphoreGive(_mutex);
    }

    return (uint16_t)((rx_buffer[0] << 8) | rx_buffer[1]); /* full 16-bit frame */
}

uint16_t AS5047P::readRegister(uint16_t reg_address)
{
    return readRegisterRaw(reg_address) & 0x3FFF; /* mask to 14-bit data */
}

void AS5047P::dumpDiagnostics(const char *label)
{
    uint16_t angle_raw = readRegisterRaw(AS5047P_REG_ANGLE); /* 0x3FFF */
    uint16_t mag_raw   = readRegisterRaw(0x3FFD);            /* MAG */
    uint16_t errfl     = readRegister(0x0001);               /* ERRFL */

    uint8_t  parity = (uint8_t)((angle_raw >> 15) & 0x1);
    uint8_t  ef     = (uint8_t)((angle_raw >> 14) & 0x1);
    uint16_t angle  = angle_raw & 0x3FFF;
    uint16_t mag    = mag_raw & 0x3FFF;

    ESP_LOGI(TAG, "%s: angle=%u(0x%04X) EF=%u par=%u MAG=%u(0x%04X) ERRFL=0x%04X",
             label, angle, angle, ef, parity, mag, mag, errfl);
}

uint16_t AS5047P::readRawPosition()
{
    return readRegister(AS5047P_REG_ANGLE);
}

float AS5047P::getSensorAngle()
{
    uint16_t raw_angle = readRawPosition();
    return (((int)(raw_angle & 0x3FFF)) * 360.0f / 16383.0f) * (PI / 180.0f);
}
