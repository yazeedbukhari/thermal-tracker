/* Declarations for the per-axis Kalman prediction/filter logic. */
#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>

typedef struct {
    float x[2];  /* state: [position, velocity]              */
    float P[4];  /* 2x2 covariance, row-major [P00 P01 P10 P11] */
    float Q_pos; /* process noise - position                 */
    float Q_vel; /* process noise - velocity                 */
    float R;     /* measurement noise                        */
    uint8_t initialized;
} KalmanAxis;

/* Reset filter state and set noise parameters */
void Kalman_Init(KalmanAxis *kf, float q_pos, float q_vel, float r);

/* Predict-only step - advance state by dt seconds */
void Kalman_Predict(KalmanAxis *kf, float dt);

/* Predict + measurement update - call on sensor frames */
void Kalman_Update(KalmanAxis *kf, float measurement, float dt);

/* Read filtered position (pixels) */
float Kalman_GetPosition(const KalmanAxis *kf);

/* Read estimated velocity (pixels/s) */
float Kalman_GetVelocity(const KalmanAxis *kf);

#endif /* INC_KALMAN_H_ */
