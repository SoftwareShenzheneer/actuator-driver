#include "foc.h"

#include <stdlib.h>

foc_t *foc_create(const foc_ops_t *ops, void *ctx)
{
    if (ops == NULL) {
        return NULL;
    }

    foc_t *f = (foc_t *)calloc(1, sizeof(*f));
    if (f == NULL) {
        return NULL;
    }

    f->ops = ops;
    f->ctx = ctx;
    return f;
}

void foc_destroy(foc_t *f)
{
    free(f);
}

int foc_init(foc_t *f)
{
    if (f == NULL || f->ops == NULL || f->ops->init == NULL) {
        return -1;
    }
    return f->ops->init(f->ctx);
}

int foc_present(foc_t *f, foc_motor_t motor)
{
    if (f == NULL || f->ops == NULL || f->ops->present == NULL) {
        return 0;
    }
    return f->ops->present(f->ctx, motor);
}

void foc_set_velocity(foc_t *f, foc_motor_t motor, float rad_s)
{
    if (f == NULL || f->ops == NULL || f->ops->set_velocity == NULL) {
        return;
    }
    f->ops->set_velocity(f->ctx, motor, rad_s);
}

float foc_get_velocity(foc_t *f, foc_motor_t motor)
{
    if (f == NULL || f->ops == NULL || f->ops->get_velocity == NULL) {
        return 0.0f;
    }
    return f->ops->get_velocity(f->ctx, motor);
}

float foc_trigger_to_velocity(uint8_t trigger)
{
    return ((float)trigger / FOC_TRIGGER_MAX) * FOC_MAX_VELOCITY_RAD_S;
}
