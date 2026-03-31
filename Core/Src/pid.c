/*
 * pid.c — Generic PID controller (one instance per axis)
 *
 * Converts pixel-space error (centroid offset from frame center) into a servo
 * angle delta. Supports integral anti-windup and an external velocity estimate
 * for the derivative term once the Kalman filter is active.
 *
 * Owner: Yazeed
 */

#include "pid.h"

/* ------------------------------------------------------------------ helpers */

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* ------------------------------------------------------------------ public  */

void PID_Init(PID_Instance *pid, float kp, float ki, float kd, float integral_limit,
              float output_limit, float dead_band)
{
    pid->Kp             = kp;
    pid->Ki             = ki;
    pid->Kd             = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->dead_band      = dead_band;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->initialized    = 0;
}

float PID_Update(PID_Instance *pid, float error, float dt, float velocity_est, uint8_t use_ext_vel)
{
    /* Dead-band: suppress output for small errors to avoid servo hunting */
    if (error > -pid->dead_band && error < pid->dead_band) {
        pid->prev_error  = error;
        pid->initialized = 1;
        return 0.0f;
    }

    /* Proportional */
    float p_term = pid->Kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += error * dt;
    pid->integral = clampf(pid->integral, -pid->integral_limit, pid->integral_limit);
    float i_term  = pid->Ki * pid->integral;

    /* Derivative
     * On the very first call prev_error is 0, which would produce a large
     * spike. Skip the D term that tick and seed prev_error instead.
     *
     * Once the Kalman filter is running the caller may pass a smoothed
     * velocity estimate; otherwise we fall back to finite difference.      */
    float d_term = 0.0f;

    if (pid->initialized) {
        float derivative;

        if (use_ext_vel) {
            /* External estimate already in pixels/s — negate because a
             * positive velocity means the error is shrinking, so we want
             * to damp (oppose) it.                                         */
            derivative = -velocity_est;
        } else {
            /* Finite difference: rate of change of error                   */
            derivative = (dt > 0.0f) ? ((error - pid->prev_error) / dt) : 0.0f;
        }

        d_term = pid->Kd * derivative;
    }

    pid->prev_error  = error;
    pid->initialized = 1;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    return clampf(output, -pid->output_limit, pid->output_limit);
}

void PID_Reset(PID_Instance *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->initialized = 0;
}
