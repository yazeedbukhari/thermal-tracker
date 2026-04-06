/* PID controller implementation for axis control error correction. */
#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

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
    float p_term;
    float i_term;
    float d_term = 0.0f;

    /* skip tiny error so servo dont shake */
    if (error > -pid->dead_band && error < pid->dead_band) {
        pid->prev_error  = error;
        pid->initialized = 1;
        return 0.0f;
    }

    p_term = pid->Kp * error;

    pid->integral += error * dt;
    pid->integral = clampf(pid->integral, -pid->integral_limit, pid->integral_limit);
    i_term = pid->Ki * pid->integral;

    if (pid->initialized) {
        float derivative;

        if (use_ext_vel) {
            derivative = -velocity_est;
        } else {
            derivative = (dt > 0.0f) ? ((error - pid->prev_error) / dt) : 0.0f;
        }

        d_term = pid->Kd * derivative;
    }

    pid->prev_error  = error;
    pid->initialized = 1;

    return clampf(p_term + i_term + d_term, -pid->output_limit, pid->output_limit);
}

void PID_Reset(PID_Instance *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->initialized = 0;
}
