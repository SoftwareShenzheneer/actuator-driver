/*
 * DualShock 4 report codec (portable core).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "dualshock4_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Parse a DualShock 4 input report into a normalized state.
 *
 *   report_id : HID report id (0x01 minimal or 0x11 full)
 *   data/len  : report payload WITHOUT the report id
 *
 * Returns 0 on success, non-zero if the report is not a recognized input report.
 */
int dualshock4_report_parse(uint8_t report_id, const uint8_t *data, size_t len, dualshock4_state_t *out);

/* Build a DualShock 4 output report payload (DUALSHOCK4_OUTPUT_REPORT_LEN bytes,
 * report id excluded) from `output`. This includes the CRC32 the controller
 * validates. */
void dualshock4_report_build_output(const dualshock4_output_t *output, uint8_t *payload);

#ifdef __cplusplus
}
#endif
