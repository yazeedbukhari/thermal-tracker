/*
 * kalman.h — Per-axis Kalman filter (constant-velocity model)
 *
 * Sits between centroid extraction and the PID controller. State vector per
 * axis is [position, velocity]^T. Scalar math on Cortex-M4F hardware FPU.
 *
 * Sensor delivers centroids at 10 Hz. The filter runs predict steps at 40 Hz
 * (25 ms tick) so the PID gets smooth inter-frame estimates. Measurement
 * updates occur only on sensor frames (every 4th tick).
 *
 * Outputs filtered position + estimated velocity per axis per tick.
 * Velocity feeds directly into the PID derivative term (derivative-on-
 * measurement), eliminating derivative kick and noise amplification.
 *
 * Public API:  Kalman_Init, Kalman_Predict, Kalman_Update,
 *              Kalman_GetPosition, Kalman_GetVelocity
 *
 * Owner: Yazeed
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>

typedef struct {
    float x[2];  /* state: [position, velocity]              */
    float P[4];  /* 2x2 covariance, row-major [P00 P01 P10 P11] */
    float Q_pos; /* process noise — position                 */
    float Q_vel; /* process noise — velocity                 */
    float R;     /* measurement noise                        */
    uint8_t initialized;
} KalmanAxis;

/* Reset filter state and set noise parameters. */
void Kalman_Init(KalmanAxis *kf, float q_pos, float q_vel, float r);

/* Predict-only step — advance state by dt seconds. */
void Kalman_Predict(KalmanAxis *kf, float dt);

/* Predict + measurement update — call on sensor frames. */
void Kalman_Update(KalmanAxis *kf, float measurement, float dt);

/* Read filtered position (pixels). */
float Kalman_GetPosition(const KalmanAxis *kf);

/* Read estimated velocity (pixels/s). */
float Kalman_GetVelocity(const KalmanAxis *kf);

#endif /* INC_KALMAN_H_ */
