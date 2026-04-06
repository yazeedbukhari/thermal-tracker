#include "kalman.h"


void Kalman_Init(KalmanAxis *kf, float q_pos, float q_vel, float r)
{
    kf->x[0] = 0.0f; /* position */
    kf->x[1] = 0.0f; /* velocity */

    // Initial covariance — large diagonal, no cross-correlation
    kf->P[0] = 1.0f; /* P00 */
    kf->P[1] = 0.0f; /* P01 */
    kf->P[2] = 0.0f; /* P10 */
    kf->P[3] = 1.0f; /* P11 */

    kf->Q_pos       = q_pos;
    kf->Q_vel       = q_vel;
    kf->R           = r;
    kf->initialized = 0U;
}

void Kalman_Predict(KalmanAxis *kf, float dt)
{
    kf->x[0] += kf->x[1] * dt;

    float p00 = kf->P[0];
    float p01 = kf->P[1];
    float p10 = kf->P[2];
    float p11 = kf->P[3];

    kf->P[0] = p00 + dt * (p10 + p01) + dt * dt * p11 + kf->Q_pos;
    kf->P[1] = p01 + dt * p11;
    kf->P[2] = p10 + dt * p11;
    kf->P[3] = p11 + kf->Q_vel;
}

void Kalman_Update(KalmanAxis *kf, float measurement, float dt)
{
    if (kf->initialized == 0U) {
        kf->x[0]        = measurement;
        kf->x[1]        = 0.0f;
        kf->initialized = 1U;
        return;
    }

    Kalman_Predict(kf, dt);

    float y = measurement - kf->x[0];

    float S = kf->P[0] + kf->R;

    float K0 = kf->P[0] / S;
    float K1 = kf->P[2] / S;

    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    float p00 = kf->P[0];
    float p01 = kf->P[1];

    kf->P[0] = (1.0f - K0) * p00;
    kf->P[1] = (1.0f - K0) * p01;
    kf->P[2] = kf->P[2] - K1 * p00;
    kf->P[3] = kf->P[3] - K1 * p01;
}

float Kalman_GetPosition(const KalmanAxis *kf)
{
    return kf->x[0];
}

float Kalman_GetVelocity(const KalmanAxis *kf)
{
    return kf->x[1];
}
