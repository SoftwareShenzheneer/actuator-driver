/*
 * DualSense transport backend for ESP32 Bluetooth Classic (BTC).
 */
#pragma once

#include "dualsense_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return the singleton ESP32 Bluetooth Classic transport for a DualSense.
 * Call once after bluetooth_service_init(). */
dualsense_transport_t *dualsense_esp32_btc_get_default(void);

#ifdef __cplusplus
}
#endif
