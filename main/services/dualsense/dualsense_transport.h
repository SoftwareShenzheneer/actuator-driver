/*
 * Transport abstraction (portable core).
 *
 * A platform backend (e.g. services/dualsense/esp32/) implements the operations
 * below; the portable core only calls through them. To port the controller
 * handling to another MCU you implement `dualsense_transport_ops_t` on the new
 * stack and reuse the rest of the core unchanged.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque transport instance. */
typedef struct dualsense_transport dualsense_transport_t;

typedef enum {
    DUALSENSE_CONN_DISCONNECTED = 0,
    DUALSENSE_CONN_CONNECTING,
    DUALSENSE_CONN_CONNECTED,
    DUALSENSE_CONN_ERROR,
} dualsense_conn_state_t;

/* The transport reports results back to the service through this sink.
 * `ctx` is whatever the service passed in; the transport only forwards it. */
typedef struct {
    /* A new input report arrived. `report_id` is the HID report id (0x01 for
     * DualSense); `data`/`len` are the payload WITHOUT the report id. The data
     * is only valid for the duration of the call. */
    void (*on_input)(void *ctx, uint8_t report_id, const uint8_t *data, size_t len);
    void (*on_conn)(void *ctx, dualsense_conn_state_t state);
    void *ctx;
} dualsense_sink_t;

typedef struct {
    /* Begin searching for and connecting to a controller. Must return promptly;
     * completion is reported through `sink->on_conn`. */
    int (*open)(dualsense_transport_t *t, const dualsense_sink_t *sink);
    int (*close)(dualsense_transport_t *t);
    /* Send an OUTPUT report payload (DUALSENSE_OUTPUT_REPORT_LEN bytes, report
     * id excluded). Returns 0 on success. */
    int (*send_report)(dualsense_transport_t *t, const uint8_t *data, size_t len);
} dualsense_transport_ops_t;

struct dualsense_transport {
    const dualsense_transport_ops_t *ops;
    void *ctx;   /* backend private data */
};

/* Construct/destroy a transport around an ops table (used by backends). */
dualsense_transport_t *dualsense_transport_create(const dualsense_transport_ops_t *ops, void *ctx);
void dualsense_transport_destroy(dualsense_transport_t *t);

#ifdef __cplusplus
}
#endif
