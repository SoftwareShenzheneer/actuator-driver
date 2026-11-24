#include "host_protocol.h"

/* 8-bit XOR checksum over the frame bytes from TYPE (inclusive) through the end
 * of the payload. SOF and the checksum byte itself are excluded. */
static uint8_t host_protocol_checksum(const uint8_t *msg, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= msg[i];
    }
    return sum;
}

size_t host_protocol_build_status(uint8_t *msg, const host_status_payload_t *status)
{
    if (msg == NULL || status == NULL) {
        return 0;
    }

    msg[HOST_PROTOCOL_OFF_SOF]  = HOST_PROTOCOL_SOF;
    msg[HOST_PROTOCOL_OFF_TYPE] = HOST_PROTOCOL_TYPE_STATUS;
    msg[HOST_PROTOCOL_OFF_LEN]  = HOST_STATUS_PAYLOAD_LEN;

    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_OFF_CONN] = status->conn;

    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_OFF_VEL_LEFT + 0] =
        (uint8_t)(status->vel_left & 0xFF);
    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_OFF_VEL_LEFT + 1] =
        (uint8_t)((status->vel_left >> 8) & 0xFF);
    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_OFF_VEL_RIGHT + 0] =
        (uint8_t)(status->vel_right & 0xFF);
    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_OFF_VEL_RIGHT + 1] =
        (uint8_t)((status->vel_right >> 8) & 0xFF);

    msg[HOST_PROTOCOL_OFF_DATA + HOST_STATUS_PAYLOAD_LEN] =
        host_protocol_checksum(&msg[HOST_PROTOCOL_OFF_TYPE],
                               HOST_PROTOCOL_OFF_DATA + HOST_STATUS_PAYLOAD_LEN - HOST_PROTOCOL_OFF_TYPE);

    return HOST_PROTOCOL_STATUS_FRAME_LEN;
}
