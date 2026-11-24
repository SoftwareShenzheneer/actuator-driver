/*
 * FOC backend for ESP32 using Espressif's esp_simplefoc component.
 */
#pragma once

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Create an ESP32 SimpleFOC-backed FOC instance. Lightweight; call foc_init()
 * afterwards to initialize the motors, run alignment, and start the control
 * loop. Returns NULL on error. */
foc_t *foc_esp32_create(void);

#ifdef __cplusplus
}
#endif
