#include "dualsense_transport.h"

#include <stdlib.h>

dualsense_transport_t *dualsense_transport_create(const dualsense_transport_ops_t *ops, void *ctx)
{
    if (ops == NULL) {
        return NULL;
    }

    dualsense_transport_t *t = (dualsense_transport_t *)calloc(1, sizeof(*t));
    if (t == NULL) {
        return NULL;
    }

    t->ops = ops;
    t->ctx = ctx;
    return t;
}

void dualsense_transport_destroy(dualsense_transport_t *t)
{
    free(t);
}
