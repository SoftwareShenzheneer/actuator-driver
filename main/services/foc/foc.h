/*
 * Field-Oriented Control (FOC) motor service (portable core).
 *
 * This is the actuator side of the firmware: it turns velocity commands into
 * motor torque through FOC. The portable core defines the interface and the
 * gamepad -> velocity mapping; a platform backend (services/foc/esp32) drives
 * the actual motor hardware. As with the other services, only the backend may
 * include platform headers, so this core can move to another MCU unchanged.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Which physical motor a command targets. The DualShock 4 R2 trigger drives the
 * right motor and L2 drives the left motor. */
typedef enum {
    FOC_MOTOR_RIGHT = 0,
    FOC_MOTOR_LEFT  = 1,
    FOC_MOTOR_COUNT = 2,
} foc_motor_t;

/* Opaque FOC instance. */
typedef struct foc foc_t;

/* Backend operations. A platform module (services/foc/esp32) implements these;
 * the portable core only calls through them. */
typedef struct {
    /* Initialize the hardware, run sensor alignment, and start the control
     * loop. Blocking. Returns 0 on success. */
    int   (*init)(void *ctx);
    /* Non-zero if the motor was detected and aligned. */
    int   (*present)(void *ctx, foc_motor_t motor);
    /* Set the closed-loop velocity target (rad/s). Non-blocking. */
    void  (*set_velocity)(void *ctx, foc_motor_t motor, float rad_s);
    /* Measured shaft velocity (rad/s). */
    float (*get_velocity)(void *ctx, foc_motor_t motor);
} foc_ops_t;

struct foc {
    const foc_ops_t *ops;
    void *ctx;
};

/* Wrap an ops table (used by platform backends). Returns NULL on error. */
foc_t *foc_create(const foc_ops_t *ops, void *ctx);
void foc_destroy(foc_t *f);

/* Convenience wrappers that dispatch through the ops table. */
int   foc_init(foc_t *f);
int   foc_present(foc_t *f, foc_motor_t motor);
void  foc_set_velocity(foc_t *f, foc_motor_t motor, float rad_s);
float foc_get_velocity(foc_t *f, foc_motor_t motor);

/* Gamepad analog triggers are 0..255. The trigger value maps linearly to a
 * velocity target between 0 and FOC_MAX_VELOCITY_RAD_S (10 rev/s). */
#define FOC_TRIGGER_MAX         255.0f
#define FOC_MAX_SPEED_REV_PER_S 4.0f    /* ~240 RPM: max the AS5600 + 1 kHz loop tracks cleanly */
#define FOC_MAX_VELOCITY_RAD_S  (FOC_MAX_SPEED_REV_PER_S * 2.0f * 3.14159265f)

float foc_trigger_to_velocity(uint8_t trigger);

#ifdef __cplusplus
}
#endif
