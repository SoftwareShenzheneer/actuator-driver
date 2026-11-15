/*
 * DualSense report codec (portable core).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "dualsense_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Parse a DualSense input report into a normalized state.
 *
 *   report_id : HID report id (DUALSENSE_INPUT_REPORT_ID for the standard input)
 *   data/len  : report payload WITHOUT the report id
 *
 * Returns 0 on success, non-zero if the report is not a recognized input report.
 */
int dualsense_report_parse(uint8_t report_id, const uint8_t *data, size_t len, dualsense_state_t *out);

/* Build a DualSense output report payload (DUALSENSE_OUTPUT_REPORT_LEN bytes,
 * report id excluded) from `output`. */
void dualsense_report_build_output(const dualsense_output_t *output, uint8_t *payload);

#ifdef __cplusplus
}
#endif
