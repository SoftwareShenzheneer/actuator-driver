/*
 * Common gamepad interface (portable core).
 *
 * The DualSense and DualShock 4 modules each expose their own state structs and
 * connection enums. This module normalizes both behind one interface so the rest
 * of the firmware (host link, actuator layer) does not care which controller is
 * attached.
 */
#pragma once

#include <stdint.h>

#include "dualsense_types.h"
#include "dualsense_transport.h"
#include "dualshock4_types.h"
#include "dualshock4_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GAMEPAD_CONN_DISCONNECTED = 0,
    GAMEPAD_CONN_CONNECTING,
    GAMEPAD_CONN_CONNECTED,
    GAMEPAD_CONN_ERROR,
} gamepad_conn_state_t;

/* Normalized analog inputs shared by both controllers.
 * Sticks: 0..255 with 128 == center. Triggers: 0 (released) .. 255 (pressed). */
typedef struct {
    uint8_t left_stick_x;
    uint8_t left_stick_y;
    uint8_t right_stick_x;
    uint8_t right_stick_y;
    uint8_t trigger_l2;
    uint8_t trigger_r2;
} gamepad_analog_t;

/* Simulator motor control value derived from the analog inputs.
 *
 * Placeholder mapping: right trigger == throttle (0..255). When the actuator
 * mapping is defined, swap this for a bidirectional scheme (R2 forward / L2
 * reverse) or a stick axis — both triggers and sticks are available in
 * gamepad_analog_t. */
uint8_t gamepad_motor_value(const gamepad_analog_t *analog);

/* Adapt connection state from each controller to the common enum. */
gamepad_conn_state_t gamepad_conn_from_dualsense(dualsense_conn_state_t s);
gamepad_conn_state_t gamepad_conn_from_dualshock4(dualshock4_conn_state_t s);

/* Copy the analog inputs from each controller's state into the common struct. */
void gamepad_analog_from_dualsense(const dualsense_state_t *src, gamepad_analog_t *dst);
void gamepad_analog_from_dualshock4(const dualshock4_state_t *src, gamepad_analog_t *dst);

#ifdef __cplusplus
}
#endif
