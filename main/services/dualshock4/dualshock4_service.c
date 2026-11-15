#include "dualshock4_service.h"
#include "dualshock4_report.h"

#include <stdlib.h>
#include <string.h>

struct dualshock4_service {
    dualshock4_transport_t *transport;
    dualshock4_callbacks_t cbs;
    dualshock4_sink_t      sink;
    dualshock4_conn_state_t conn_state;
};

static void service_on_input(void *ctx, uint8_t report_id, const uint8_t *data, size_t len)
{
    dualshock4_service_t *s = (dualshock4_service_t *)ctx;
    dualshock4_state_t state;

    if (dualshock4_report_parse(report_id, data, len, &state) == 0) {
        if (s->cbs.report_cb != NULL) {
            s->cbs.report_cb(&state, s->cbs.user);
        }
    }
}

static void service_on_conn(void *ctx, dualshock4_conn_state_t state)
{
    dualshock4_service_t *s = (dualshock4_service_t *)ctx;
    s->conn_state = state;
    if (s->cbs.conn_cb != NULL) {
        s->cbs.conn_cb(state, s->cbs.user);
    }
}

dualshock4_service_t *dualshock4_service_create(dualshock4_transport_t *transport,
                                                const dualshock4_callbacks_t *cbs)
{
    if (transport == NULL || transport->ops == NULL) {
        return NULL;
    }

    dualshock4_service_t *s = (dualshock4_service_t *)calloc(1, sizeof(*s));
    if (s == NULL) {
        return NULL;
    }

    s->transport = transport;
    if (cbs != NULL) {
        s->cbs = *cbs;
    }

    s->conn_state = DUALSHOCK4_CONN_DISCONNECTED;
    s->sink.on_input = service_on_input;
    s->sink.on_conn  = service_on_conn;
    s->sink.ctx      = s;

    return s;
}

int dualshock4_service_start(dualshock4_service_t *service)
{
    if (service == NULL) {
        return -1;
    }

    service->conn_state = DUALSHOCK4_CONN_CONNECTING;
    return service->transport->ops->open(service->transport, &service->sink);
}

int dualshock4_service_stop(dualshock4_service_t *service)
{
    if (service == NULL) {
        return -1;
    }

    return service->transport->ops->close(service->transport);
}

int dualshock4_service_set_output(dualshock4_service_t *service, const dualshock4_output_t *output)
{
    if (service == NULL || output == NULL) {
        return -1;
    }

    uint8_t payload[DUALSHOCK4_OUTPUT_REPORT_LEN];
    dualshock4_report_build_output(output, payload);
    return service->transport->ops->send_report(service->transport, payload, sizeof(payload));
}

void dualshock4_service_destroy(dualshock4_service_t *service)
{
    if (service == NULL) {
        return;
    }

    if (service->transport != NULL) {
        service->transport->ops->close(service->transport);
    }
    free(service);
}
