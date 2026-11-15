/*
 * DualSense controller types (portable core).
 *
 * Everything under services/dualsense/ except the esp32/ subdirectory must NOT
 * include any ESP-IDF or RTOS headers, so it can be reused unchanged on a
 * different MCU. Only the transport backend is platform-specific.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DualSense HID report layout over Bluetooth Classic.
 *
 * The DualSense is a HID peripheral on Bluetooth Classic (BR/EDR):
 *   - INPUT  report 0x01: the "minimal" report — 1 byte report id + 9 byte
 *     payload (sticks, triggers, buttons). The full report is 0x31 (78 bytes)
 *     and is not used here.
 *   - OUTPUT report 0x31: 48 byte payload (the report id is sent out-of-band by
 *     the transport, so it is excluded from the payload lengths below)
 */
#define DUALSENSE_INPUT_REPORT_ID      0x01u
#define DUALSENSE_INPUT_REPORT_LEN     9u    /* payload, report id excluded */
#define DUALSENSE_OUTPUT_REPORT_ID     0x31u
#define DUALSENSE_OUTPUT_REPORT_LEN    48u   /* payload, report id excluded */

/* Digital buttons (bit flags, in the order the service reports them). */
typedef enum {
    DUALSENSE_BTN_SQUARE     = (1u << 0),
    DUALSENSE_BTN_CROSS      = (1u << 1),
    DUALSENSE_BTN_CIRCLE     = (1u << 2),
    DUALSENSE_BTN_TRIANGLE   = (1u << 3),
    DUALSENSE_BTN_L1         = (1u << 4),
    DUALSENSE_BTN_R1         = (1u << 5),
    DUALSENSE_BTN_L2         = (1u << 6),
    DUALSENSE_BTN_R2         = (1u << 7),
    DUALSENSE_BTN_CREATE     = (1u << 8),
    DUALSENSE_BTN_OPTIONS    = (1u << 9),
    DUALSENSE_BTN_L3         = (1u << 10),
    DUALSENSE_BTN_R3         = (1u << 11),
    DUALSENSE_BTN_PS         = (1u << 12),
    DUALSENSE_BTN_TOUCHPAD   = (1u << 13),
    DUALSENSE_BTN_MUTE       = (1u << 14),
    DUALSENSE_BTN_DPAD_UP    = (1u << 16),
    DUALSENSE_BTN_DPAD_DOWN  = (1u << 17),
    DUALSENSE_BTN_DPAD_LEFT  = (1u << 18),
    DUALSENSE_BTN_DPAD_RIGHT = (1u << 19),
} dualsense_button_t;

typedef enum {
    DUALSENSE_BATTERY_UNKNOWN = 0,
    DUALSENSE_BATTERY_DISCHARGING,
    DUALSENSE_BATTERY_CHARGING,
    DUALSENSE_BATTERY_FULL,
} dualsense_battery_status_t;

/* Normalized controller state (what the application consumes). */
typedef struct {
    uint32_t buttons;              /* dualsense_button_t flags */

    /* Analog sticks: 0..255, 128 == center. */
    uint8_t left_stick_x;
    uint8_t left_stick_y;
    uint8_t right_stick_x;
    uint8_t right_stick_y;

    /* Analog triggers: 0 (released) .. 255 (fully pressed). */
    uint8_t trigger_l2;
    uint8_t trigger_r2;

    /* Battery: 0..100 percent, 0xFF when unknown. */
    uint8_t battery_level;
    dualsense_battery_status_t battery_status;

    /* Touch / motion data are intentionally left out until the report offsets
     * have been validated against real hardware. */
} dualsense_state_t;

/* Output we can push to the controller (rumble + lightbar). */
typedef struct {
    uint8_t rumble_right;          /* low-frequency motor (whole body), 0..255 */
    uint8_t rumble_left;           /* high-frequency motor (grips), 0..255 */
    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
} dualsense_output_t;

#ifdef __cplusplus
}
#endif
