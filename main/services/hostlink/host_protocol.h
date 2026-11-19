/*
 * Host-link protocol (portable core).
 *
 * Defines the serialized frame pushed to the host over UART. The whole message
 * is a flat byte array, so the host decodes it positionally as
 * msg[0], msg[1], msg[x], ...
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frame header byte. */
#define HOST_PROTOCOL_SOF 0xA5u

/* Message types. */
#define HOST_PROTOCOL_TYPE_STATUS 0x01u

/* Frame layout (byte indices). */
#define HOST_PROTOCOL_OFF_SOF   0
#define HOST_PROTOCOL_OFF_TYPE  1
#define HOST_PROTOCOL_OFF_LEN   2
#define HOST_PROTOCOL_OFF_DATA  3

/* Payload layout for HOST_PROTOCOL_TYPE_STATUS. */
#define HOST_STATUS_PAYLOAD_LEN   2u
#define HOST_STATUS_OFF_CONN      0
#define HOST_STATUS_OFF_MOTOR     1

/* SOF + TYPE + LEN + payload + trailing checksum. */
#define HOST_PROTOCOL_STATUS_FRAME_LEN \
    (HOST_PROTOCOL_OFF_DATA + HOST_STATUS_PAYLOAD_LEN + 1u)

/* The status "object" that is serialized into the payload. Add fields here (and
 * bump HOST_STATUS_PAYLOAD_LEN) as the object grows. */
typedef struct {
    uint8_t conn;         /* connection state (see gamepad_conn_state_t) */
    uint8_t motor_value;  /* simulator motor control, 0..255 */
} host_status_payload_t;

/* Serialize a status message into msg[] (at least HOST_PROTOCOL_STATUS_FRAME_LEN
 * bytes). Returns the frame length written, or 0 on error. */
size_t host_protocol_build_status(uint8_t *msg, const host_status_payload_t *status);

#ifdef __cplusplus
}
#endif
