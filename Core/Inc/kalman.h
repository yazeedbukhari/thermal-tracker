/*
 * kalman.h — Per-axis Kalman filter (constant-velocity model, CMSIS-DSP)
 *
 * Sits between centroid extraction and the PID controller. State vector per
 * axis is [position, velocity]^T. Uses CMSIS-DSP arm_mat_*_f32 routines for
 * predict/update matrix math on the Cortex-M4F hardware FPU.
 *
 * Outputs filtered position + estimated velocity per axis per frame.
 * Velocity feeds directly into the PID derivative term (derivative-on-
 * measurement), eliminating derivative kick and noise amplification.
 *
 * Public API:  Kalman_Init, Kalman_Update
 *
 * Owner:
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_



#endif /* INC_KALMAN_H_ */
