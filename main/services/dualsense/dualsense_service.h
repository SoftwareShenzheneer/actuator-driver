/*
 * DualSense service (portable core).
 *
 * Glues the transport backend to the report codec and exposes a simple
 * callback API to the application.
 */
#pragma once

#include "dualsense_types.h"
#include "dualsense_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dualsense_service dualsense_service_t;

typedef void (*dualsense_report_cb_t)(const dualsense_state_t *state, void *user);
typedef void (*dualsense_conn_cb_t)(dualsense_conn_state_t state, void *user);

typedef struct {
    dualsense_report_cb_t report_cb;
    dualsense_conn_cb_t   conn_cb;
    void                 *user;
} dualsense_callbacks_t;

/* Create a service bound to a transport. Returns NULL on error. */
dualsense_service_t *dualsense_service_create(dualsense_transport_t *transport,
                                              const dualsense_callbacks_t *cbs);

/* Start connecting. Non-blocking; progress is reported through `conn_cb`. */
int dualsense_service_start(dualsense_service_t *service);
int dualsense_service_stop(dualsense_service_t *service);

/* Push a new output (rumble/LED) to the controller. */
int dualsense_service_set_output(dualsense_service_t *service, const dualsense_output_t *output);

void dualsense_service_destroy(dualsense_service_t *service);

#ifdef __cplusplus
}
#endif
