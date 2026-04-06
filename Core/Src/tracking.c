/* Converts thermal detections into smoothed pan/tilt motion commands. */
#include "tracking.h"
#include "servo.h"

#define SENSOR_CENTER_X      3.5f
#define SENSOR_CENTER_Y      3.5f

#define TRACK_PROC_SCALE     (31.0f / 7.0f) /* map 8x8 coordinates into 32x32 domain */
#define TRACK_CENTER_X32     15.5f
#define TRACK_CENTER_Y32     15.5f
#define TRACK_HALF_RANGE32   15.5f

#define TRACK_DEADBAND_X32   0.45f /* sub-cell deadband in 32x32 coordinates */
#define TRACK_DEADBAND_Y32   0.45f

#define TRACK_STEP_MIN_DEG   2.1f
#define TRACK_STEP_MAX_DEG   8.4f
#define TRACK_MOTION_FULL32  3.2f

#define TRACK_VEL_ALPHA      0.52f
#define TRACK_CMD_ALPHA_SLOW 0.50f
#define TRACK_CMD_ALPHA_FAST 0.90f

#define TRACK_CONF_ALPHA     0.28f
#define TRACK_CONF_MARGIN_FULL_C 3.8f
#define TRACK_CONF_SIZE_FULL_PX  10.0f

/* Lost-target behavior:
 * 1) Coast briefly in last known direction to recover target.
 * 2) If still lost for >2.2 s, enter scan mode (pan sweep + tilt stepping). */
#define TRACK_COAST_MS            1000U
#define TRACK_SCAN_START_MS       2200U
#define TRACK_SCAN_PAN_DEG_PER_S  45.0f
#define TRACK_SCAN_TILT_DEG_PER_S 18.0f
#define TRACK_SCAN_PAN_MIN        12.0f
#define TRACK_SCAN_PAN_MAX        168.0f
#define TRACK_SCAN_TILT_MIN       50.0f
#define TRACK_SCAN_TILT_MAX       130.0f

static uint8_t tracking_enabled = 1U;
static uint8_t has_prev_centroid = 0U;
static float prev_cx32 = TRACK_CENTER_X32;
static float prev_cy32 = TRACK_CENTER_Y32;
static float motion_ewma = 0.0f;
static float conf_ewma = 0.0f;
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
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static float normalize_deadband(float err, float deadband, float full_scale)
{
    float a = absf_local(err);
    if (a <= deadband) {
        return 0.0f;
    }

    float n = (a - deadband) / (full_scale - deadband);
    n = clampf_local(n, 0.0f, 1.0f);

    /* Slight cubic emphasis for sub-cell aiming near center. */
    n = (0.55f * n) + (0.45f * (n * n * n));
    return (err < 0.0f) ? -n : n;
}

void Tracking_Init(void)
{
    tracking_enabled = 1U;
    has_prev_centroid = 0U;
    has_prev_cmd = 0U;
    motion_ewma = 0.0f;
    conf_ewma = 0.0f;
    lost_since_ms = HAL_GetTick();
    last_update_ms = lost_since_ms;
    scan_pan_dir = 1U;
    scan_tilt_dir = 1U;
}

void Tracking_Enable(uint8_t enable)
{
    tracking_enabled = (enable != 0U) ? 1U : 0U;
}

void Tracking_ResetSearchTimer(void)
{
    uint32_t now = HAL_GetTick();
    lost_since_ms = now;
    last_update_ms = now;
    has_prev_centroid = 0U;
    has_prev_cmd = 0U;
    motion_ewma = 0.0f;
    conf_ewma = 0.0f;
    prev_pan_cmd = 0.0f;
    prev_tilt_cmd = 0.0f;
}

void Tracking_UpdateFromDetection(const ThermalDetection *det)
{
    if ((det == 0) || (tracking_enabled == 0U)) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt_s = 0.1f;
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

        if ((lost_ms < TRACK_COAST_MS) && (has_prev_cmd != 0U)) {
            float fade = 1.0f - ((float)lost_ms / (float)TRACK_COAST_MS);
            if (fade < 0.0f)
                fade = 0.0f;
            Servo_SetPan(Servo_GetPan() + (prev_pan_cmd * fade));
            Servo_SetTilt(Servo_GetTilt() + (prev_tilt_cmd * fade));
            has_prev_centroid = 0U;
            return;
        }

        if (lost_ms >= TRACK_SCAN_START_MS) {
            float pan = Servo_GetPan();
            float tilt = Servo_GetTilt();
            float pan_step = TRACK_SCAN_PAN_DEG_PER_S * dt_s;
            float tilt_step = TRACK_SCAN_TILT_DEG_PER_S * dt_s;
            if (pan_step < 0.2f)
                pan_step = 0.2f;
            if (tilt_step < 0.1f)
                tilt_step = 0.1f;

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
            return;
        }

        has_prev_centroid = 0U;
        return;
    }

    lost_since_ms = 0U;

    /* Work in the same 32x32 space used by thermal processing. */
    float cx32 = det->centroid_x * TRACK_PROC_SCALE;
    float cy32 = det->centroid_y * TRACK_PROC_SCALE;
    float ex32 = cx32 - TRACK_CENTER_X32;
    float ey32 = cy32 - TRACK_CENTER_Y32;

    float nx = normalize_deadband(ex32, TRACK_DEADBAND_X32, TRACK_HALF_RANGE32);
    float ny = normalize_deadband(ey32, TRACK_DEADBAND_Y32, TRACK_HALF_RANGE32);

    float motion_raw = 0.0f;
    if (has_prev_centroid != 0U) {
        float dx = cx32 - prev_cx32;
        float dy = cy32 - prev_cy32;
        motion_raw = absf_local(dx) + absf_local(dy);
    }
    prev_cx32 = cx32;
    prev_cy32 = cy32;
    has_prev_centroid = 1U;

    motion_ewma += TRACK_VEL_ALPHA * (motion_raw - motion_ewma);
    float motion_norm = clampf_local(motion_ewma / TRACK_MOTION_FULL32, 0.0f, 1.0f);

    /* Confidence combines thermal margin and blob size. */
    float temp_margin = det->max_temp_c - det->threshold_c;
    float margin_norm = clampf_local(temp_margin / TRACK_CONF_MARGIN_FULL_C, 0.0f, 1.0f);
    float size_norm = clampf_local((float)det->hot_count / TRACK_CONF_SIZE_FULL_PX, 0.0f, 1.0f);
    float conf_now = 0.65f * margin_norm + 0.35f * size_norm;
    conf_ewma += TRACK_CONF_ALPHA * (conf_now - conf_ewma);

    float confidence_gate = 0.35f + (0.65f * conf_ewma);

    float step_limit = TRACK_STEP_MIN_DEG + (TRACK_STEP_MAX_DEG - TRACK_STEP_MIN_DEG) * motion_norm;
    step_limit *= confidence_gate;

    float cmd_alpha = TRACK_CMD_ALPHA_SLOW + (TRACK_CMD_ALPHA_FAST - TRACK_CMD_ALPHA_SLOW) * motion_norm;
    cmd_alpha = clampf_local(cmd_alpha * (0.65f + (0.35f * conf_ewma)), 0.25f, 0.95f);

    /* Orientation mapping: pan angle decreases to move right; tilt increases to move down. */
    float raw_pan = -nx * step_limit;
    float raw_tilt = ny * step_limit;

    if (has_prev_cmd == 0U) {
        prev_pan_cmd = raw_pan;
        prev_tilt_cmd = raw_tilt;
        has_prev_cmd = 1U;
    } else {
        prev_pan_cmd += cmd_alpha * (raw_pan - prev_pan_cmd);
        prev_tilt_cmd += cmd_alpha * (raw_tilt - prev_tilt_cmd);
    }

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




