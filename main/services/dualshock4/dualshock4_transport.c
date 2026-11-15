#include "dualshock4_transport.h"

#include <stdlib.h>

dualshock4_transport_t *dualshock4_transport_create(const dualshock4_transport_ops_t *ops, void *ctx)
{
    if (ops == NULL) {
        return NULL;
    }

    dualshock4_transport_t *t = (dualshock4_transport_t *)calloc(1, sizeof(*t));
    if (t == NULL) {
        return NULL;
    }

    t->ops = ops;
    t->ctx = ctx;
    return t;
}

void dualshock4_transport_destroy(dualshock4_transport_t *t)
{
    free(t);
}
