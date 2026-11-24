#include "gamepad.h"

gamepad_conn_state_t gamepad_conn_from_dualsense(dualsense_conn_state_t s)
{
    switch (s) {
    case DUALSENSE_CONN_CONNECTING: return GAMEPAD_CONN_CONNECTING;
    case DUALSENSE_CONN_CONNECTED:  return GAMEPAD_CONN_CONNECTED;
    case DUALSENSE_CONN_ERROR:      return GAMEPAD_CONN_ERROR;
    default:                        return GAMEPAD_CONN_DISCONNECTED;
    }
}

gamepad_conn_state_t gamepad_conn_from_dualshock4(dualshock4_conn_state_t s)
{
    switch (s) {
    case DUALSHOCK4_CONN_CONNECTING: return GAMEPAD_CONN_CONNECTING;
    case DUALSHOCK4_CONN_CONNECTED:  return GAMEPAD_CONN_CONNECTED;
    case DUALSHOCK4_CONN_ERROR:      return GAMEPAD_CONN_ERROR;
    default:                         return GAMEPAD_CONN_DISCONNECTED;
    }
}

void gamepad_analog_from_dualsense(const dualsense_state_t *src, gamepad_analog_t *dst)
{
    if (src == NULL || dst == NULL) {
        return;
    }

    dst->left_stick_x  = src->left_stick_x;
    dst->left_stick_y  = src->left_stick_y;
    dst->right_stick_x = src->right_stick_x;
    dst->right_stick_y = src->right_stick_y;
    dst->trigger_l2    = src->trigger_l2;
    dst->trigger_r2    = src->trigger_r2;
}

void gamepad_analog_from_dualshock4(const dualshock4_state_t *src, gamepad_analog_t *dst)
{
    if (src == NULL || dst == NULL) {
        return;
    }

    dst->left_stick_x  = src->left_stick_x;
    dst->left_stick_y  = src->left_stick_y;
    dst->right_stick_x = src->right_stick_x;
    dst->right_stick_y = src->right_stick_y;
    dst->trigger_l2    = src->trigger_l2;
    dst->trigger_r2    = src->trigger_r2;
}
