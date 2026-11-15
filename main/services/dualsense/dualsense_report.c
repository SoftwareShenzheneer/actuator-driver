#include "dualsense_report.h"

#include <string.h>

/*
 * DualSense over Bluetooth Classic sends a MINIMAL input report on report id
 * 0x01 (9-byte payload) with the essential gamepad state. The FULL report
 * (report id 0x31, 78 bytes) additionally carries touch/gyro/battery, but is
 * not used here.
 *
 * 9-byte payload layout (verified against the Linux kernel driver
 * drivers/hid/hid-playstation.c, struct dualsense_input_report):
 *   [0] left stick X     (0..255, 128 = center)
 *   [1] left stick Y
 *   [2] right stick X
 *   [3] right stick Y
 *   [4] L2 trigger       (0..255)
 *   [5] R2 trigger
 *   [6] sequence number
 *   [7] buttons[0]: d-pad hat (bits 0-3) | Square(4) Cross(5) Circle(6) Triangle(7)
 *   [8] buttons[1]: L1(0) R1(1) L2(2) R2(3) Create(4) Options(5) L3(6) R3(7)
 *
 * PS / touchpad / mute and battery live in the full 0x31 report, so they are
 * not available here (battery reports 0xFF / UNKNOWN).
 */

/* D-pad hat-switch values (buttons[0] bits 0-3); 8 = released. */
#define HAT_NORTH       0
#define HAT_NORTH_EAST  1
#define HAT_EAST        2
#define HAT_SOUTH_EAST  3
#define HAT_SOUTH       4
#define HAT_SOUTH_WEST  5
#define HAT_WEST        6
#define HAT_NORTH_WEST  7

static void set_dpad(uint32_t *buttons, uint8_t hat)
{
    if (hat == HAT_NORTH || hat == HAT_NORTH_EAST || hat == HAT_NORTH_WEST) {
        *buttons |= DUALSENSE_BTN_DPAD_UP;
    }
    if (hat == HAT_EAST || hat == HAT_NORTH_EAST || hat == HAT_SOUTH_EAST) {
        *buttons |= DUALSENSE_BTN_DPAD_RIGHT;
    }
    if (hat == HAT_SOUTH || hat == HAT_SOUTH_EAST || hat == HAT_SOUTH_WEST) {
        *buttons |= DUALSENSE_BTN_DPAD_DOWN;
    }
    if (hat == HAT_WEST || hat == HAT_SOUTH_WEST || hat == HAT_NORTH_WEST) {
        *buttons |= DUALSENSE_BTN_DPAD_LEFT;
    }
}

int dualsense_report_parse(uint8_t report_id, const uint8_t *data, size_t len, dualsense_state_t *out)
{
    if (out == NULL) {
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->battery_level = 0xFF;
    out->battery_status = DUALSENSE_BATTERY_UNKNOWN;

    if (report_id != DUALSENSE_INPUT_REPORT_ID || data == NULL || len < DUALSENSE_INPUT_REPORT_LEN) {
        return -1;
    }

    out->left_stick_x  = data[0];
    out->left_stick_y  = data[1];
    out->right_stick_x = data[2];
    out->right_stick_y = data[3];
    out->trigger_l2    = data[4];
    out->trigger_r2    = data[5];

    uint8_t b0 = data[7];
    uint8_t b1 = data[8];

    set_dpad(&out->buttons, b0 & 0x0F);

    if (b0 & (1 << 4)) out->buttons |= DUALSENSE_BTN_SQUARE;
    if (b0 & (1 << 5)) out->buttons |= DUALSENSE_BTN_CROSS;
    if (b0 & (1 << 6)) out->buttons |= DUALSENSE_BTN_CIRCLE;
    if (b0 & (1 << 7)) out->buttons |= DUALSENSE_BTN_TRIANGLE;

    if (b1 & (1 << 0)) out->buttons |= DUALSENSE_BTN_L1;
    if (b1 & (1 << 1)) out->buttons |= DUALSENSE_BTN_R1;
    if (b1 & (1 << 2)) out->buttons |= DUALSENSE_BTN_L2;
    if (b1 & (1 << 3)) out->buttons |= DUALSENSE_BTN_R2;
    if (b1 & (1 << 4)) out->buttons |= DUALSENSE_BTN_CREATE;
    if (b1 & (1 << 5)) out->buttons |= DUALSENSE_BTN_OPTIONS;
    if (b1 & (1 << 6)) out->buttons |= DUALSENSE_BTN_L3;
    if (b1 & (1 << 7)) out->buttons |= DUALSENSE_BTN_R3;

    return 0;
}

void dualsense_report_build_output(const dualsense_output_t *output, uint8_t *payload)
{
    if (output == NULL || payload == NULL) {
        return;
    }

    memset(payload, 0, DUALSENSE_OUTPUT_REPORT_LEN);

    /* Flag bytes that enable rumble and the lightbar. */
    payload[0] = 0x03;   /* 0x01 | 0x02 : valid rumble/haptics data */
    payload[1] = 0x06;   /* 0x02 | 0x04 : valid lightbar data */
    payload[2] = 0x17;   /* 0x01|0x02|0x04|0x10 : player indicator LEDs */

    /* Rumble motors. */
    payload[3] = output->rumble_right;   /* low-frequency (whole body) */
    payload[4] = output->rumble_left;    /* high-frequency (grips) */

    /* Lightbar RGB. */
    payload[39] = output->led_red;
    payload[40] = output->led_green;
    payload[41] = output->led_blue;
}
