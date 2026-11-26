set(sources
    "main.c"
    "services/bluetooth/bluetooth_service.c"
    "services/dualsense/dualsense_transport.c"
    "services/dualsense/dualsense_report.c"
    "services/dualsense/dualsense_service.c"
    "services/dualsense/esp32/dualsense_esp32_btc.c"
    "services/dualshock4/dualshock4_transport.c"
    "services/dualshock4/dualshock4_report.c"
    "services/dualshock4/dualshock4_service.c"
    "services/dualshock4/esp32/dualshock4_esp32_btc.c"
    "services/gamepad/gamepad.c"
    "services/hostlink/host_protocol.c"
    "services/hostlink/esp32/hostlink_esp32_uart.c"
    "services/foc/foc.c"
    "services/foc/esp32/foc_esp32.cpp"
    "services/foc/esp32/as5047p_sensor.cpp"
    "services/foc/esp32/local_lowside_cs.cpp"
)

set(include_dirs
    "."
    "services/bluetooth"
    "services/dualsense"
    "services/dualsense/esp32"
    "services/dualshock4"
    "services/dualshock4/esp32"
    "services/gamepad"
    "services/hostlink"
    "services/hostlink/esp32"
    "services/foc"
    "services/foc/esp32"
)

set(requires
    esp_hid
    nvs_flash
    esp_driver_uart
    esp_driver_gpio
    esp_driver_mcpwm
    esp_driver_i2c
    esp_driver_spi
)

# The AS5047P SPI encoder is an alternative to the AS5600 (FOC) encoders. It is
# built only when selected in menuconfig (Encoder -> AS5047P).
if(CONFIG_ENCODER_AS5047P)
    list(APPEND sources
        "services/gpio/gpio_service.c"
        "services/rotary-encoder/rotary_encoder.c"
        "services/rotary-encoder/esp32/rotary_encoder_esp32.c"
    )
    list(APPEND include_dirs
        "services/gpio"
        "services/rotary-encoder"
        "services/rotary-encoder/esp32"
    )
    list(APPEND requires esp_driver_spi)
endif()

idf_component_register(
    SRCS ${sources}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)
