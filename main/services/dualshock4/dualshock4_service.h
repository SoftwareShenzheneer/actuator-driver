/*
 * DualShock 4 service (portable core).
 *
 * Glues the transport backend to the report codec and exposes a simple
 * callback API to the application.
 */
#pragma once

#include "dualshock4_types.h"
#include "dualshock4_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dualshock4_service dualshock4_service_t;

typedef void (*dualshock4_report_cb_t)(const dualshock4_state_t *state, void *user);
typedef void (*dualshock4_conn_cb_t)(dualshock4_conn_state_t state, void *user);

typedef struct {
    dualshock4_report_cb_t report_cb;
    dualshock4_conn_cb_t   conn_cb;
    void                  *user;
} dualshock4_callbacks_t;

/* Create a service bound to a transport. Returns NULL on error. */
dualshock4_service_t *dualshock4_service_create(dualshock4_transport_t *transport,
                                                const dualshock4_callbacks_t *cbs);

/* Start connecting. Non-blocking; progress is reported through `conn_cb`. */
int dualshock4_service_start(dualshock4_service_t *service);
int dualshock4_service_stop(dualshock4_service_t *service);

/* Push a new output (rumble/LED) to the controller. */
int dualshock4_service_set_output(dualshock4_service_t *service, const dualshock4_output_t *output);

void dualshock4_service_destroy(dualshock4_service_t *service);

#ifdef __cplusplus
}
#endif
