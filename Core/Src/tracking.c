/*
 * tracking.c - Thermal centroid to pan/tilt servo tracker
 */

#include "tracking.h"
#include "servo.h"

#define SENSOR_CENTER_X      3.5f
#define SENSOR_CENTER_Y      3.5f

#define TRACK_DEADBAND_X     0.10f  /* pixels */
#define TRACK_DEADBAND_Y     0.10f  /* pixels */
#define SENSOR_HALF_RANGE    3.5f   /* from center (3.5) to edge */

#define TRACK_STEP_MIN_DEG   2.4f   /* calm target -> still responsive */
#define TRACK_STEP_MAX_DEG   8.0f   /* fast target -> more aggressive */
#define TRACK_MOTION_FULL    0.7f   /* reaches fast mode sooner */

#define TRACK_VEL_ALPHA      0.55f  /* motion EWMA */
#define TRACK_CMD_ALPHA_SLOW 0.55f  /* output filter when target is slow */
#define TRACK_CMD_ALPHA_FAST 0.92f  /* output filter when target is fast */

/* Lost-target behavior:
 * 1) Coast briefly in last known direction to recover target.
 * 2) If still lost for >5s, enter scan mode (pan sweep + tilt stepping). */
#define TRACK_COAST_MS            700U
#define TRACK_SCAN_START_MS       3000U
#define TRACK_SCAN_PAN_DEG_PER_S  45.0f
#define TRACK_SCAN_TILT_DEG_PER_S 18.0f
#define TRACK_SCAN_PAN_MIN        12.0f
#define TRACK_SCAN_PAN_MAX        168.0f
#define TRACK_SCAN_TILT_MIN       50.0f
#define TRACK_SCAN_TILT_MAX       130.0f

static uint8_t tracking_enabled = 1U;
static uint8_t has_prev_centroid = 0U;
static float prev_cx = SENSOR_CENTER_X;
static float prev_cy = SENSOR_CENTER_Y;
static float motion_ewma = 0.0f;
static float prev_pan_cmd = 0.0f;
static float prev_tilt_cmd = 0.0f;
static uint8_t has_prev_cmd = 0U;
static uint32_t lost_since_ms = 0U;
static uint32_t last_update_ms = 0U;
static uint8_t scan_pan_dir = 1U;   /* 1: increasing pan, 0: decreasing pan */
static uint8_t scan_tilt_dir = 1U;  /* 1: increasing tilt, 0: decreasing tilt */

static float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float normalize_deadband(float err, float deadband, float full_scale)
{
    float a = absf_local(err);
    if (a <= deadband) {
        return 0.0f;
    }

    /* Joystick-like response: dead zone then normalized 0..1 output. */
    float n = (a - deadband) / (full_scale - deadband);
    n = clampf_local(n, 0.0f, 1.0f);

    /* Keep center smooth but avoid sluggish mid-range response. */
    n = 0.65f * n + 0.35f * (n * n);
    return (err < 0.0f) ? -n : n;
}

void Tracking_Init(void)
{
    tracking_enabled = 1U;
    has_prev_centroid = 0U;
    has_prev_cmd = 0U;
    motion_ewma = 0.0f;
    lost_since_ms = HAL_GetTick();
    last_update_ms = lost_since_ms;
    scan_pan_dir = 1U;
    scan_tilt_dir = 1U;
}

void Tracking_Enable(uint8_t enable)
{
    tracking_enabled = (enable != 0U) ? 1U : 0U;
}

void Tracking_UpdateFromDetection(const ThermalDetection *det)
{
    if ((det == 0) || (tracking_enabled == 0U)) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt_s = 0.1f; /* fallback for first update */
    if (last_update_ms != 0U) {
        uint32_t dt_ms = now - last_update_ms;
        if (dt_ms > 0U) {
            dt_s = (float)dt_ms * 0.001f;
        }
    }
    last_update_ms = now;

    if (det->target_found == 0U) {
        uint32_t lost_ms;
        if (lost_since_ms == 0U) {
            lost_since_ms = now;
        }
        lost_ms = now - lost_since_ms;

        /* Short-term recovery: keep moving in last known direction, then fade out. */
        if ((lost_ms < TRACK_COAST_MS) && (has_prev_cmd != 0U)) {
            float fade = 1.0f - ((float)lost_ms / (float)TRACK_COAST_MS);
            if (fade < 0.0f) fade = 0.0f;
            Servo_SetPan(Servo_GetPan() + (prev_pan_cmd * fade));
            Servo_SetTilt(Servo_GetTilt() + (prev_tilt_cmd * fade));
            has_prev_centroid = 0U;
            return;
        }

        /* Long-term recovery: autonomous scan mode after 5 seconds lost. */
        if (lost_ms >= TRACK_SCAN_START_MS) {
            float pan = Servo_GetPan();
            float tilt = Servo_GetTilt();
            float pan_step = TRACK_SCAN_PAN_DEG_PER_S * dt_s;
            float tilt_step = TRACK_SCAN_TILT_DEG_PER_S * dt_s;
            if (pan_step < 0.2f) pan_step = 0.2f; /* keep motion visible if dt jitters */
            if (tilt_step < 0.1f) tilt_step = 0.1f;

            if (scan_pan_dir != 0U) {
                pan += pan_step;
                if (pan >= TRACK_SCAN_PAN_MAX) {
                    pan = TRACK_SCAN_PAN_MAX;
                    scan_pan_dir = 0U;
                }
            } else {
                pan -= pan_step;
                if (pan <= TRACK_SCAN_PAN_MIN) {
                    pan = TRACK_SCAN_PAN_MIN;
                    scan_pan_dir = 1U;
                }
            }

            /* Continuous tilt sweep during scan mode. */
            if (scan_tilt_dir != 0U) {
                tilt += tilt_step;
                if (tilt >= TRACK_SCAN_TILT_MAX) {
                    tilt = TRACK_SCAN_TILT_MAX;
                    scan_tilt_dir = 0U;
                }
            } else {
                tilt -= tilt_step;
                if (tilt <= TRACK_SCAN_TILT_MIN) {
                    tilt = TRACK_SCAN_TILT_MIN;
                    scan_tilt_dir = 1U;
                }
            }

            Servo_SetPan(pan);
            Servo_SetTilt(tilt);
            has_prev_centroid = 0U;
            has_prev_cmd = 0U;
            return;
        }

        /* Between coast and scan: hold while waiting for scan timeout. */
        has_prev_centroid = 0U;
        return;
    }

    /* Target reacquired: return to normal tracking behavior. */
    lost_since_ms = 0U;

    /* Error in 8x8 sensor coordinates. */
    float ex = det->centroid_x - SENSOR_CENTER_X;
    float ey = det->centroid_y - SENSOR_CENTER_Y;

    float nx = normalize_deadband(ex, TRACK_DEADBAND_X, SENSOR_HALF_RANGE);
    float ny = normalize_deadband(ey, TRACK_DEADBAND_Y, SENSOR_HALF_RANGE);

    /* Estimate target motion speed (pixels/frame), smoothed by EWMA. */
    float motion_raw = 0.0f;
    if (has_prev_centroid != 0U) {
        float dx = det->centroid_x - prev_cx;
        float dy = det->centroid_y - prev_cy;
        motion_raw = absf_local(dx) + absf_local(dy); /* no sqrt needed */
    }
    prev_cx = det->centroid_x;
    prev_cy = det->centroid_y;
    has_prev_centroid = 1U;

    motion_ewma += TRACK_VEL_ALPHA * (motion_raw - motion_ewma);
    float motion_norm = clampf_local(motion_ewma / TRACK_MOTION_FULL, 0.0f, 1.0f);

    /* Faster target motion -> larger step budget and less command smoothing. */
    float step_limit = TRACK_STEP_MIN_DEG +
        (TRACK_STEP_MAX_DEG - TRACK_STEP_MIN_DEG) * motion_norm;
    float cmd_alpha = TRACK_CMD_ALPHA_SLOW +
        (TRACK_CMD_ALPHA_FAST - TRACK_CMD_ALPHA_SLOW) * motion_norm;

    /*
     * Pan orientation in this project:
     * - lower angle -> right
     * - higher angle -> left
     * If target is to the right (ex > 0), decrease pan angle.
     */
    float raw_pan = -nx * step_limit;

    /*
     * Tilt orientation in this project:
     * - lower angle -> up
     * - higher angle -> down
     * If target is lower in image (ey > 0), increase tilt angle.
     */
    float raw_tilt = ny * step_limit;

    if (has_prev_cmd == 0U) {
        prev_pan_cmd = raw_pan;
        prev_tilt_cmd = raw_tilt;
        has_prev_cmd = 1U;
    } else {
        prev_pan_cmd += cmd_alpha * (raw_pan - prev_pan_cmd);
        prev_tilt_cmd += cmd_alpha * (raw_tilt - prev_tilt_cmd);
    }

    /* Remember current tracking direction so scan mode starts by continuing
     * in the same pan/tilt direction when target is later lost. */
    if (prev_pan_cmd > 0.0f) {
        scan_pan_dir = 1U;
    } else if (prev_pan_cmd < 0.0f) {
        scan_pan_dir = 0U;
    }
    if (prev_tilt_cmd > 0.0f) {
        scan_tilt_dir = 1U;
    } else if (prev_tilt_cmd < 0.0f) {
        scan_tilt_dir = 0U;
    }

    Servo_SetPan(Servo_GetPan() + prev_pan_cmd);
    Servo_SetTilt(Servo_GetTilt() + prev_tilt_cmd);
}
