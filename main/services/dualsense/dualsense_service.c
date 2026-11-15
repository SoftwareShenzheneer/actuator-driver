#include "dualsense_service.h"
#include "dualsense_report.h"

#include <stdlib.h>
#include <string.h>

struct dualsense_service {
    dualsense_transport_t *transport;
    dualsense_callbacks_t cbs;
    dualsense_sink_t      sink;
    dualsense_conn_state_t conn_state;
};

static void service_on_input(void *ctx, uint8_t report_id, const uint8_t *data, size_t len)
{
    dualsense_service_t *s = (dualsense_service_t *)ctx;
    dualsense_state_t state;

    if (dualsense_report_parse(report_id, data, len, &state) == 0) {
        if (s->cbs.report_cb != NULL) {
            s->cbs.report_cb(&state, s->cbs.user);
        }
    }
}

static void service_on_conn(void *ctx, dualsense_conn_state_t state)
{
    dualsense_service_t *s = (dualsense_service_t *)ctx;
    s->conn_state = state;
    if (s->cbs.conn_cb != NULL) {
        s->cbs.conn_cb(state, s->cbs.user);
    }
}

dualsense_service_t *dualsense_service_create(dualsense_transport_t *transport,
                                              const dualsense_callbacks_t *cbs)
{
    if (transport == NULL || transport->ops == NULL) {
        return NULL;
    }

    dualsense_service_t *s = (dualsense_service_t *)calloc(1, sizeof(*s));
    if (s == NULL) {
        return NULL;
    }

    s->transport = transport;
    if (cbs != NULL) {
        s->cbs = *cbs;
    }

    s->conn_state = DUALSENSE_CONN_DISCONNECTED;
    s->sink.on_input = service_on_input;
    s->sink.on_conn  = service_on_conn;
    s->sink.ctx      = s;

    return s;
}

int dualsense_service_start(dualsense_service_t *service)
{
    if (service == NULL) {
        return -1;
    }

    service->conn_state = DUALSENSE_CONN_CONNECTING;
    return service->transport->ops->open(service->transport, &service->sink);
}

int dualsense_service_stop(dualsense_service_t *service)
{
    if (service == NULL) {
        return -1;
    }

    return service->transport->ops->close(service->transport);
}

int dualsense_service_set_output(dualsense_service_t *service, const dualsense_output_t *output)
{
    if (service == NULL || output == NULL) {
        return -1;
    }

    uint8_t payload[DUALSENSE_OUTPUT_REPORT_LEN];
    dualsense_report_build_output(output, payload);
    return service->transport->ops->send_report(service->transport, payload, sizeof(payload));
}

void dualsense_service_destroy(dualsense_service_t *service)
{
    if (service == NULL) {
        return;
    }

    if (service->transport != NULL) {
        service->transport->ops->close(service->transport);
    }
    free(service);
}
