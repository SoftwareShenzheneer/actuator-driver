/*
 * Transport abstraction (portable core).
 *
 * A platform backend (e.g. services/dualshock4/esp32/) implements the
 * operations below; the portable core only calls through them. To port the
 * controller handling to another MCU you implement `dualshock4_transport_ops_t`
 * on the new stack and reuse the rest of the core unchanged.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque transport instance. */
typedef struct dualshock4_transport dualshock4_transport_t;

typedef enum {
    DUALSHOCK4_CONN_DISCONNECTED = 0,
    DUALSHOCK4_CONN_CONNECTING,
    DUALSHOCK4_CONN_CONNECTED,
    DUALSHOCK4_CONN_ERROR,
} dualshock4_conn_state_t;

/* The transport reports results back to the service through this sink.
 * `ctx` is whatever the service passed in; the transport only forwards it. */
typedef struct {
    /* A new input report arrived. `report_id` is the HID report id (0x01 for
     * the minimal report, 0x11 for the full report); `data`/`len` are the
     * payload WITHOUT the report id. The data is only valid for the duration
     * of the call. */
    void (*on_input)(void *ctx, uint8_t report_id, const uint8_t *data, size_t len);
    void (*on_conn)(void *ctx, dualshock4_conn_state_t state);
    void *ctx;
} dualshock4_sink_t;

typedef struct {
    /* Begin searching for and connecting to a controller. Must return promptly;
     * completion is reported through `sink->on_conn`. */
    int (*open)(dualshock4_transport_t *t, const dualshock4_sink_t *sink);
    int (*close)(dualshock4_transport_t *t);
    /* Send an OUTPUT report payload (DUALSHOCK4_OUTPUT_REPORT_LEN bytes, report
     * id excluded). Returns 0 on success. */
    int (*send_report)(dualshock4_transport_t *t, const uint8_t *data, size_t len);
} dualshock4_transport_ops_t;

struct dualshock4_transport {
    const dualshock4_transport_ops_t *ops;
    void *ctx;   /* backend private data */
};

/* Construct/destroy a transport around an ops table (used by backends). */
dualshock4_transport_t *dualshock4_transport_create(const dualshock4_transport_ops_t *ops, void *ctx);
void dualshock4_transport_destroy(dualshock4_transport_t *t);

#ifdef __cplusplus
}
#endif
