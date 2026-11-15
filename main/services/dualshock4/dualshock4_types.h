/*
 * DualShock 4 controller types (portable core).
 *
 * Everything under services/dualshock4/ except the esp32/ subdirectory must NOT
 * include any ESP-IDF or RTOS headers, so it can be reused unchanged on a
 * different MCU. Only the transport backend is platform-specific.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DualShock 4 HID report layout over Bluetooth Classic.
 *
 * The DualShock 4 is a HID peripheral on Bluetooth Classic (BR/EDR):
 *   - INPUT  report 0x01: a "minimal" 10-byte report (1 byte report id +
 *     9 byte payload) carrying sticks/buttons/triggers. Some third-party pads
 *     only ever send this.
 *   - INPUT  report 0x11: the full 78-byte report (report id + 77 byte
 *     payload), adding touch/gyro/battery and a trailing CRC32.
 *   - OUTPUT report 0x11: 78-byte report (report id + 77 byte payload) with a
 *     trailing CRC32 that the controller validates.
 *
 * The report id is sent out-of-band by the transport, so all lengths below
 * exclude it.
 */
#define DUALSHOCK4_INPUT_REPORT_MINIMAL_ID    0x01u
#define DUALSHOCK4_INPUT_REPORT_MINIMAL_LEN   9u    /* payload, report id excluded */
#define DUALSHOCK4_INPUT_REPORT_FULL_ID       0x11u
#define DUALSHOCK4_INPUT_REPORT_FULL_LEN      77u   /* payload, report id excluded */
#define DUALSHOCK4_OUTPUT_REPORT_ID           0x11u
#define DUALSHOCK4_OUTPUT_REPORT_LEN          77u   /* payload, report id excluded */

/* Digital buttons (bit flags, in the order the service reports them).
 * Bit positions match the DualSense module where the buttons correspond, so an
 * actuator layer can treat the two controllers uniformly. */
typedef enum {
    DUALSHOCK4_BTN_SQUARE     = (1u << 0),
    DUALSHOCK4_BTN_CROSS      = (1u << 1),
    DUALSHOCK4_BTN_CIRCLE     = (1u << 2),
    DUALSHOCK4_BTN_TRIANGLE   = (1u << 3),
    DUALSHOCK4_BTN_L1         = (1u << 4),
    DUALSHOCK4_BTN_R1         = (1u << 5),
    DUALSHOCK4_BTN_L2         = (1u << 6),
    DUALSHOCK4_BTN_R2         = (1u << 7),
    DUALSHOCK4_BTN_SHARE      = (1u << 8),
    DUALSHOCK4_BTN_OPTIONS    = (1u << 9),
    DUALSHOCK4_BTN_L3         = (1u << 10),
    DUALSHOCK4_BTN_R3         = (1u << 11),
    DUALSHOCK4_BTN_PS         = (1u << 12),
    DUALSHOCK4_BTN_TOUCHPAD   = (1u << 13),
    DUALSHOCK4_BTN_DPAD_UP    = (1u << 16),
    DUALSHOCK4_BTN_DPAD_DOWN  = (1u << 17),
    DUALSHOCK4_BTN_DPAD_LEFT  = (1u << 18),
    DUALSHOCK4_BTN_DPAD_RIGHT = (1u << 19),
} dualshock4_button_t;

typedef enum {
    DUALSHOCK4_BATTERY_UNKNOWN = 0,
    DUALSHOCK4_BATTERY_DISCHARGING,
    DUALSHOCK4_BATTERY_CHARGING,
    DUALSHOCK4_BATTERY_FULL,
} dualshock4_battery_status_t;

/* Normalized controller state (what the application consumes). */
typedef struct {
    uint32_t buttons;              /* dualshock4_button_t flags */

    /* Analog sticks: 0..255, 128 == center. */
    uint8_t left_stick_x;
    uint8_t left_stick_y;
    uint8_t right_stick_x;
    uint8_t right_stick_y;

    /* Analog triggers: 0 (released) .. 255 (fully pressed). */
    uint8_t trigger_l2;
    uint8_t trigger_r2;

    /* Battery: 0..100 percent, 0xFF when unknown (minimal 0x01 reports carry
     * no battery data; only the full 0x11 report does). */
    uint8_t battery_level;
    dualshock4_battery_status_t battery_status;

    /* Touch / motion data are intentionally left out for now. */
} dualshock4_state_t;

/* Output we can push to the controller (rumble + lightbar). */
typedef struct {
    uint8_t motor_left;            /* strong rumble: low-frequency, whole body */
    uint8_t motor_right;           /* weak rumble: high-frequency, grips */
    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
} dualshock4_output_t;

#ifdef __cplusplus
}
#endif
