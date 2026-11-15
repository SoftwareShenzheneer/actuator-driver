/*
 * DualShock 4 transport backend for ESP32 Bluetooth Classic (BTC).
 */
#pragma once

#include "dualshock4_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return the singleton ESP32 Bluetooth Classic transport for a DualShock 4.
 * Call once after bluetooth_service_init(). */
dualshock4_transport_t *dualshock4_esp32_btc_get_default(void);

#ifdef __cplusplus
}
#endif
