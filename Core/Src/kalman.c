/*
 * kalman.c — Per-axis Kalman filter (constant-velocity model, CMSIS-DSP)
 *
 * Sits between centroid extraction and the PID controller. Uses CMSIS-DSP
 * arm_mat_*_f32 routines for predict/update matrix math. Outputs filtered
 * position + estimated velocity; velocity feeds the PID derivative term.
 *
 * Owner:
 */

#include "kalman.h"
