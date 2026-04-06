/* Declarations for PID controller structures and update functions. */
#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prev_error;

    float integral_limit;
    float output_limit;
    float dead_band;

    uint8_t initialized;
} PID_Instance;

/* Configure gains and limits for one axis. */
void PID_Init(PID_Instance *pid, float kp, float ki, float kd, float integral_limit,
              float output_limit, float dead_band);

/* Run one control cycle and return the servo angle delta. */
float PID_Update(PID_Instance *pid, float error, float dt, float velocity_est, uint8_t use_ext_vel);

/* Zero the stored state without changing gains. */
void PID_Reset(PID_Instance *pid);

#endif /* INC_PID_H_ */
