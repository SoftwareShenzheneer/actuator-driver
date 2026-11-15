#include "dualshock4_report.h"

#include <stdbool.h>
#include <string.h>

/*
 * DualShock 4 over Bluetooth Classic. Layouts verified against the Linux
 * kernel driver drivers/hid/hid-playstation.c (struct dualshock4_input_report_*,
 * struct dualshock4_output_report_*, dualshock4_parse_report()).
 *
 * Input reports (report id excluded from the byte offsets below):
 *
 *   MINIMAL report 0x01, 9-byte payload — the first 9 bytes of the full
 *   report's common section, so the gamepad fields are in the SAME order as
 *   the full report:
 *     [0] left stick X        (0..255, 128 = center)
 *     [1] left stick Y
 *     [2] right stick X
 *     [3] right stick Y
 *     [4] buttons[0]
 *     [5] buttons[1]
 *     [6] buttons[2]
 *     [7] L2 trigger          (0..255)
 *     [8] R2 trigger
 *
 *   FULL report 0x11, 77-byte payload (78 bytes incl. report id):
 *     [0..1]  reserved
 *     [2..10] same 9 gamepad bytes as above, then:
 *     [11..]  timestamp / gyro / accel / touch ...
 *     [31]    status[0]: low nibble = battery capacity (units of 10%),
 *             bit 4 = cable state (1 = USB/charging)
 *     [73..76] CRC32 (little-endian)
 *
 * NOTE: the minimal 0x01 report carries the buttons at offsets 4..6 and the
 * triggers at 7..8 — a DIFFERENT layout from the DualSense minimal 0x01 report
 * (which puts triggers at 4..5 and buttons at 7..8).
 */

/* buttons[0] */
#define BTN0_HAT_MASK     0x0F
#define BTN0_SQUARE       (1u << 4)
#define BTN0_CROSS        (1u << 5)
#define BTN0_CIRCLE       (1u << 6)
#define BTN0_TRIANGLE     (1u << 7)

/* buttons[1] */
#define BTN1_L1           (1u << 0)
#define BTN1_R1           (1u << 1)
#define BTN1_L2           (1u << 2)
#define BTN1_R2           (1u << 3)
#define BTN1_SHARE        (1u << 4)
#define BTN1_OPTIONS      (1u << 5)
#define BTN1_L3           (1u << 6)
#define BTN1_R3           (1u << 7)

/* buttons[2] */
#define BTN2_PS           (1u << 0)
#define BTN2_TOUCHPAD     (1u << 1)

/* status[0] in the full 0x11 report (payload offset 31). */
#define STATUS0_BATTERY_CAPACITY_MASK  0x0F
#define STATUS0_CABLE_STATE            (1u << 4)

/* D-pad hat-switch values (buttons[0] bits 0-3); 8 = released. */
#define HAT_NORTH       0
#define HAT_NORTH_EAST  1
#define HAT_EAST        2
#define HAT_SOUTH_EAST  3
#define HAT_SOUTH       4
#define HAT_SOUTH_WEST  5
#define HAT_WEST        6
#define HAT_NORTH_WEST  7

/* Output report (0x11) fields, as payload offsets (report id excluded). */
#define OUT_HW_CONTROL    0
#define OUT_AUDIO_CONTROL 1
#define OUT_VALID_FLAG0   2
#define OUT_MOTOR_RIGHT   5
#define OUT_MOTOR_LEFT    6
#define OUT_LED_RED       7
#define OUT_LED_GREEN     8
#define OUT_LED_BLUE      9

/* hw_control bits: tell the controller the report carries HID data + a CRC. */
#define OUT_HWCTL_HID     0x80
#define OUT_HWCTL_CRC32   0x40

/* BT poll interval (lower 6 bits of hw_control). A non-zero value is what makes
 * a genuine DS4 switch from the minimal 0x01 input report to the full 0x11 one.
 * 4 ms matches the Linux driver default (DS4_BT_DEFAULT_POLL_INTERVAL_MS). */
#define OUT_BT_POLL_INTERVAL_4MS  0x04u

/* valid_flag0 bits. */
#define OUT_VALID_MOTOR   0x01
#define OUT_VALID_LED     0x02

/* CRC32 seed byte (PS_OUTPUT_CRC32_SEED). */
#define CRC32_SEED        0xA2u

static void set_dpad(uint32_t *buttons, uint8_t hat)
{
    if (hat == HAT_NORTH || hat == HAT_NORTH_EAST || hat == HAT_NORTH_WEST) {
        *buttons |= DUALSHOCK4_BTN_DPAD_UP;
    }
    if (hat == HAT_EAST || hat == HAT_NORTH_EAST || hat == HAT_SOUTH_EAST) {
        *buttons |= DUALSHOCK4_BTN_DPAD_RIGHT;
    }
    if (hat == HAT_SOUTH || hat == HAT_SOUTH_EAST || hat == HAT_SOUTH_WEST) {
        *buttons |= DUALSHOCK4_BTN_DPAD_DOWN;
    }
    if (hat == HAT_WEST || hat == HAT_SOUTH_WEST || hat == HAT_NORTH_WEST) {
        *buttons |= DUALSHOCK4_BTN_DPAD_LEFT;
    }
}

/* Parse the 9 gamepad bytes shared by the minimal and full reports. `d` points
 * at the first byte (left stick X). */
static void parse_gamepad(const uint8_t *d, dualshock4_state_t *out)
{
    uint8_t b0 = d[4];
    uint8_t b1 = d[5];
    uint8_t b2 = d[6];

    out->left_stick_x  = d[0];
    out->left_stick_y  = d[1];
    out->right_stick_x = d[2];
    out->right_stick_y = d[3];
    out->trigger_l2    = d[7];
    out->trigger_r2    = d[8];

    set_dpad(&out->buttons, b0 & BTN0_HAT_MASK);

    if (b0 & BTN0_SQUARE)   out->buttons |= DUALSHOCK4_BTN_SQUARE;
    if (b0 & BTN0_CROSS)    out->buttons |= DUALSHOCK4_BTN_CROSS;
    if (b0 & BTN0_CIRCLE)   out->buttons |= DUALSHOCK4_BTN_CIRCLE;
    if (b0 & BTN0_TRIANGLE) out->buttons |= DUALSHOCK4_BTN_TRIANGLE;

    if (b1 & BTN1_L1)       out->buttons |= DUALSHOCK4_BTN_L1;
    if (b1 & BTN1_R1)       out->buttons |= DUALSHOCK4_BTN_R1;
    if (b1 & BTN1_L2)       out->buttons |= DUALSHOCK4_BTN_L2;
    if (b1 & BTN1_R2)       out->buttons |= DUALSHOCK4_BTN_R2;
    if (b1 & BTN1_SHARE)    out->buttons |= DUALSHOCK4_BTN_SHARE;
    if (b1 & BTN1_OPTIONS)  out->buttons |= DUALSHOCK4_BTN_OPTIONS;
    if (b1 & BTN1_L3)       out->buttons |= DUALSHOCK4_BTN_L3;
    if (b1 & BTN1_R3)       out->buttons |= DUALSHOCK4_BTN_R3;

    if (b2 & BTN2_PS)       out->buttons |= DUALSHOCK4_BTN_PS;
    if (b2 & BTN2_TOUCHPAD) out->buttons |= DUALSHOCK4_BTN_TOUCHPAD;
}

int dualshock4_report_parse(uint8_t report_id, const uint8_t *data, size_t len, dualshock4_state_t *out)
{
    if (out == NULL) {
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->battery_level = 0xFF;
    out->battery_status = DUALSHOCK4_BATTERY_UNKNOWN;

    if (data == NULL) {
        return -1;
    }

    if (report_id == DUALSHOCK4_INPUT_REPORT_MINIMAL_ID &&
        len >= DUALSHOCK4_INPUT_REPORT_MINIMAL_LEN) {
        /* Minimal 0x01: the gamepad fields start at payload offset 0. */
        parse_gamepad(data, out);
        return 0;
    }

    if (report_id == DUALSHOCK4_INPUT_REPORT_FULL_ID &&
        len >= DUALSHOCK4_INPUT_REPORT_FULL_LEN) {
        /* Full 0x11: the gamepad fields start after a 2-byte reserved header,
         * and battery lives in status[0] at payload offset 31. */
        parse_gamepad(data + 2, out);

        uint8_t status0 = data[31];
        uint8_t capacity = status0 & STATUS0_BATTERY_CAPACITY_MASK;
        bool cable = (status0 & STATUS0_CABLE_STATE) != 0;

        out->battery_level = capacity * 10;
        if (cable && capacity == 10) {
            out->battery_status = DUALSHOCK4_BATTERY_FULL;
        } else if (cable) {
            out->battery_status = DUALSHOCK4_BATTERY_CHARGING;
        } else {
            out->battery_status = DUALSHOCK4_BATTERY_DISCHARGING;
        }
        return 0;
    }

    return -1;
}

/* Standard CRC-32 (reflected, poly 0xEDB88320) — same algorithm as the Linux
 * kernel's crc32_le(), so the value matches what the DS4 expects. */
static uint32_t crc32_update(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

/* CRC over [seed byte, report_id, payload[0..len-1]]. */
static uint32_t crc32_over_report(uint8_t report_id, const uint8_t *payload, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    crc = crc32_update(crc, CRC32_SEED);
    crc = crc32_update(crc, report_id);
    for (size_t i = 0; i < len; i++) {
        crc = crc32_update(crc, payload[i]);
    }
    return crc ^ 0xFFFFFFFFu;
}

void dualshock4_report_build_output(const dualshock4_output_t *output, uint8_t *payload)
{
    if (output == NULL || payload == NULL) {
        return;
    }

    memset(payload, 0, DUALSHOCK4_OUTPUT_REPORT_LEN);

    /* Tell the controller this report has HID data and a valid CRC32. */
    payload[OUT_HW_CONTROL] = OUT_HWCTL_HID | OUT_HWCTL_CRC32 | OUT_BT_POLL_INTERVAL_4MS;

    /* Enable both rumble and lightbar, and fill them in. */
    payload[OUT_VALID_FLAG0] = OUT_VALID_MOTOR | OUT_VALID_LED;
    payload[OUT_MOTOR_RIGHT] = output->motor_right;
    payload[OUT_MOTOR_LEFT]  = output->motor_left;
    payload[OUT_LED_RED]     = output->led_red;
    payload[OUT_LED_GREEN]   = output->led_green;
    payload[OUT_LED_BLUE]    = output->led_blue;

    /* CRC32 covers everything except the last 4 bytes (the CRC itself). */
    uint32_t crc = crc32_over_report(DUALSHOCK4_OUTPUT_REPORT_ID, payload,
                                     DUALSHOCK4_OUTPUT_REPORT_LEN - 4);
    size_t crc_off = DUALSHOCK4_OUTPUT_REPORT_LEN - 4;
    payload[crc_off + 0] = (uint8_t)(crc & 0xFF);
    payload[crc_off + 1] = (uint8_t)((crc >> 8) & 0xFF);
    payload[crc_off + 2] = (uint8_t)((crc >> 16) & 0xFF);
    payload[crc_off + 3] = (uint8_t)((crc >> 24) & 0xFF);
}
