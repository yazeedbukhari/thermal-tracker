#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct {
    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* State */
    float integral;   /* Accumulated integral term                     */
    float prev_error; /* Error from the previous call (for D term)     */

    /* Anti-windup: integral is clamped to [-integral_limit, +integral_limit] */
    float integral_limit;

    /* Output clamp: final angle delta limited to [-output_limit, +output_limit] */
    float output_limit;

    /* Dead-band: errors smaller than this magnitude produce zero output */
    float dead_band;

    /* Set to 1 after the first PID_Update so the D term doesn't spike */
    uint8_t initialized;
} PID_Instance;

/*
 * PID_Init — configure gains and limits for one axis.
 *
 * @pid             Pointer to an uninitialized PID_Instance.
 * @kp / ki / kd    PID gains (angle-delta per pixel / per pixel·s / per pixel/s)
 * @integral_limit  Anti-windup clamp for the integral accumulator.
 * @output_limit    Maximum magnitude of the returned angle delta (degrees).
 * @dead_band       Error magnitude below which output is forced to zero (pixels).
 */
void PID_Init(PID_Instance *pid, float kp, float ki, float kd, float integral_limit,
              float output_limit, float dead_band);

/*
 * PID_Update — run one control cycle and return the servo angle delta.
 *
 * @pid             Pointer to an initialized PID_Instance.
 * @error           Signed pixel-space error: centroid_axis - frame_center
 *                  (positive = target is to the right / below center).
 * @dt              Time since the last call, in seconds.
 * @velocity_est    Optional external velocity estimate (pixels/s) for the D
 *                  term — pass 0.0f to use the finite-difference fallback.
 * @use_ext_vel     1 = use velocity_est for D term, 0 = finite difference.
 *
 * Returns the angle delta to add to the current servo position (degrees).
 * Positive delta moves the servo in the direction that reduces a positive error.
 */
float PID_Update(PID_Instance *pid, float error, float dt, float velocity_est, uint8_t use_ext_vel);

/*
 * PID_Reset — zero the integral and derivative state without changing gains.
 * Call this when tracking is lost so stale state doesn't jerk the servo on
 * re-acquisition.
 */
void PID_Reset(PID_Instance *pid);

#endif /* INC_PID_H_ */
