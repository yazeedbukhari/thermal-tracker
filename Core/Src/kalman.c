/*
 * kalman.c — Per-axis Kalman filter (constant-velocity model)
 *
 * 2-state filter [position, velocity] with scalar math — no CMSIS-DSP needed.
 * H = [1 0], so the update collapses to simple scalar operations on a 2x2
 * covariance matrix (4 floats).
 *
 * Owner: Yazeed
 */

#include "kalman.h"

/* ── Public API ──────────────────────────────────────────────────────────── */

void Kalman_Init(KalmanAxis *kf, float q_pos, float q_vel, float r)
{
    kf->x[0] = 0.0f;   /* position */
    kf->x[1] = 0.0f;   /* velocity */

    /* Initial covariance — large diagonal, no cross-correlation */
    kf->P[0] = 1.0f;   /* P00 */
    kf->P[1] = 0.0f;   /* P01 */
    kf->P[2] = 0.0f;   /* P10 */
    kf->P[3] = 1.0f;   /* P11 */

    kf->Q_pos = q_pos;
    kf->Q_vel = q_vel;
    kf->R     = r;
    kf->initialized = 0U;
}

/*
 * Kalman_Predict — constant-velocity state transition.
 *
 *   F = [1  dt]    x⁻ = F·x
 *       [0   1]    P⁻ = F·P·Fᵀ + Q
 */
void Kalman_Predict(KalmanAxis *kf, float dt)
{
    /* State predict */
    kf->x[0] += kf->x[1] * dt;
    /* kf->x[1] unchanged (constant velocity) */

    /* Covariance predict:  P⁻ = F·P·Fᵀ + Q
     *
     * F·P = [P00+dt*P10   P01+dt*P11]
     *       [P10          P11       ]
     *
     * (F·P)·Fᵀ = [(P00+dt*P10) + dt*(P01+dt*P11)   P01+dt*P11]
     *             [P10+dt*P11                        P11       ]
     */
    float p00 = kf->P[0];
    float p01 = kf->P[1];
    float p10 = kf->P[2];
    float p11 = kf->P[3];

    kf->P[0] = p00 + dt * (p10 + p01) + dt * dt * p11 + kf->Q_pos;
    kf->P[1] = p01 + dt * p11;
    kf->P[2] = p10 + dt * p11;
    kf->P[3] = p11 + kf->Q_vel;
}

/*
 * Kalman_Update — predict then incorporate a measurement.
 *
 * H = [1 0], so:
 *   y = z - x[0]
 *   S = P[0] + R          (scalar)
 *   K = [P[0]/S, P[2]/S]
 */
void Kalman_Update(KalmanAxis *kf, float measurement, float dt)
{
    /* On first measurement, seed the state instead of filtering. */
    if (kf->initialized == 0U) {
        kf->x[0] = measurement;
        kf->x[1] = 0.0f;
        kf->initialized = 1U;
        return;
    }

    /* Predict step */
    Kalman_Predict(kf, dt);

    /* Innovation */
    float y = measurement - kf->x[0];

    /* Innovation covariance (scalar) */
    float S = kf->P[0] + kf->R;

    /* Kalman gain */
    float K0 = kf->P[0] / S;
    float K1 = kf->P[2] / S;

    /* State update */
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    /* Covariance update:  P = (I - K·H) · P⁻
     * K·H = [K0  0]
     *       [K1  0]
     */
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
