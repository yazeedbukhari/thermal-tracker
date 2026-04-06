/* Thermal frame processing, custom upscale, and object extraction logic. */
#include "thermal.h"

#define TEMP_OFFSET_C               5.3f
#define ABSOLUTE_TARGET_CLAMP_C     32.0f
#define ABSOLUTE_MIN_TARGET_TEMP_C  24.0f
#define CONNECT_OFFSET_C            0.7f

#define THERMAL_PROC_W              32U
#define THERMAL_PROC_H              32U
#define THERMAL_PROC_PIXELS         (THERMAL_PROC_W * THERMAL_PROC_H)
#define THERMAL_PROC_MIN_HOT_PIXELS 26
#define THERMAL_PROC_BOOST_THRESHOLD_C 1.0f
#define THERMAL_PROC_BOOST_GAIN        0.48f
#define THERMAL_PROC_EDGE_GAIN         0.34f

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static uint16_t clampu16_local(int32_t v, uint16_t lo, uint16_t hi)
{
    if (v < (int32_t)lo)
        return lo;
    if (v > (int32_t)hi)
        return hi;
    return (uint16_t)v;
}

static void clear_objects(ThermalObjectsResult *out)
{
    out->avg_temp_c = 0.0f;
    out->threshold_c = 0.0f;
    out->max_temp_c = -1000.0f;
    out->total_hot_count = 0U;
    out->count = 0U;

    for (uint32_t i = 0U; i < THERMAL_MAX_OBJECTS; i++) {
        out->objects[i].valid = 0U;
        out->objects[i].min_x = -1;
        out->objects[i].max_x = -1;
        out->objects[i].min_y = -1;
        out->objects[i].max_y = -1;
        out->objects[i].centroid_x = -1.0f;
        out->objects[i].centroid_y = -1.0f;
        out->objects[i].peak_temp_c = -1000.0f;
        out->objects[i].hot_count = 0U;
    }
}

static void sort_objects_by_priority(ThermalObjectsResult *out)
{
    for (uint32_t i = 0U; i + 1U < out->count; i++) {
        for (uint32_t j = i + 1U; j < out->count; j++) {
            const ThermalObject *a = &out->objects[i];
            const ThermalObject *b = &out->objects[j];
            uint8_t swap = 0U;

            if (b->peak_temp_c > a->peak_temp_c) {
                swap = 1U;
            } else if ((b->peak_temp_c == a->peak_temp_c) && (b->hot_count > a->hot_count)) {
                swap = 1U;
            }

            if (swap != 0U) {
                ThermalObject tmp = out->objects[i];
                out->objects[i] = out->objects[j];
                out->objects[j] = tmp;
            }
        }
    }
}

static void thermal_build_proc32(const float in8x8[64], float out32[THERMAL_PROC_PIXELS], float ambient)
{
    const float sx = 7.0f / (float)(THERMAL_PROC_W - 1U);
    const float sy = 7.0f / (float)(THERMAL_PROC_H - 1U);

    /* build 32x32 map first so later logic works on smoother grid */
    for (uint16_t oy = 0U; oy < THERMAL_PROC_H; oy++) {
        float gy = ((float)oy) * sy;
        int32_t y0 = (int32_t)gy;
        int32_t y1 = (y0 < 7) ? (y0 + 1) : 7;
        float fy = gy - (float)y0;

        for (uint16_t ox = 0U; ox < THERMAL_PROC_W; ox++) {
            float gx = ((float)ox) * sx;
            int32_t x0 = (int32_t)gx;
            int32_t x1 = (x0 < 7) ? (x0 + 1) : 7;
            float fx = gx - (float)x0;

            float v00 = in8x8[(y0 * 8) + x0];
            float v10 = in8x8[(y0 * 8) + x1];
            float v01 = in8x8[(y1 * 8) + x0];
            float v11 = in8x8[(y1 * 8) + x1];

            /* Base interpolation only for trend. */
            float base = (v00 * (1.0f - fx) * (1.0f - fy)) +
                         (v10 * fx * (1.0f - fy)) +
                         (v01 * (1.0f - fx) * fy) +
                         (v11 * fx * fy);

            /* Keep sharp thermal by focusing to nearest source pixel. */
            int32_t xn = (int32_t)(gx + 0.5f);
            int32_t yn = (int32_t)(gy + 0.5f);
            uint16_t sxn = clampu16_local(xn, 0U, 7U);
            uint16_t syn = clampu16_local(yn, 0U, 7U);
            float anchor = in8x8[(syn * 8U) + sxn];

            float grad_x = v10 - v00;
            float grad_y = v01 - v00;
            float edge_strength = clampf_local((grad_x * grad_x) + (grad_y * grad_y), 0.0f, 5.0f);
            float edge_mix = clampf_local(THERMAL_PROC_EDGE_GAIN * edge_strength, 0.05f, 0.75f);

            float mixed = (base * (1.0f - edge_mix)) + (anchor * edge_mix);

            if ((anchor - ambient) > THERMAL_PROC_BOOST_THRESHOLD_C) {
                mixed += THERMAL_PROC_BOOST_GAIN * (anchor - mixed);
            }

            out32[(oy * THERMAL_PROC_W) + ox] = mixed;
        }
    }
}

static uint16_t proc_to_sensor_hotcount(uint32_t hot_px32)
{
    uint32_t scaled = (hot_px32 + 8U) / 16U;
    if (scaled > 65535U) {
        scaled = 65535U;
    }
    return (uint16_t)scaled;
}

void Thermal_DetectObjects8x8(const float frame_c[64], ThermalObjectsResult *out)
{
    if ((frame_c == 0) || (out == 0)) {
        return;
    }

    clear_objects(out);

    float sum8 = 0.0f;
    for (uint16_t i = 0U; i < 64U; i++) {
        float t = frame_c[i];
        sum8 += t;
    }

    float avg8 = sum8 / 64.0f;

    float proc32[THERMAL_PROC_PIXELS];
    thermal_build_proc32(frame_c, proc32, avg8);

    float sum32 = 0.0f;
    float max32 = -1000.0f;
    for (uint16_t i = 0U; i < THERMAL_PROC_PIXELS; i++) {
        float t = proc32[i];
        sum32 += t;
        if (t > max32) {
            max32 = t;
        }
    }

    float avg32 = sum32 / (float)THERMAL_PROC_PIXELS;
    float spread32 = max32 - avg32;

    float threshold = avg32 + TEMP_OFFSET_C + (0.18f * spread32);
    if (threshold > ABSOLUTE_TARGET_CLAMP_C) {
        threshold = ABSOLUTE_TARGET_CLAMP_C;
    }

    int effective_min_pixels = THERMAL_PROC_MIN_HOT_PIXELS;
    if (max32 > (ABSOLUTE_TARGET_CLAMP_C + 2.0f)) {
        effective_min_pixels = THERMAL_PROC_MIN_HOT_PIXELS - 8;
    }

    out->avg_temp_c = avg32;
    out->threshold_c = threshold;
    out->max_temp_c = max32;

    uint8_t hot_mask[THERMAL_PROC_PIXELS];
    uint8_t connect_mask[THERMAL_PROC_PIXELS];
    uint8_t visited[THERMAL_PROC_PIXELS];
    float connect_threshold = threshold + CONNECT_OFFSET_C;
    float connect_absolute = ABSOLUTE_MIN_TARGET_TEMP_C + CONNECT_OFFSET_C;

    for (uint16_t i = 0U; i < THERMAL_PROC_PIXELS; i++) {
        float t = proc32[i];
        hot_mask[i] = (uint8_t)(((t >= threshold) || (t >= ABSOLUTE_MIN_TARGET_TEMP_C)) ? 1U : 0U);
        connect_mask[i] =
            (uint8_t)(((t >= connect_threshold) || (t >= connect_absolute)) ? 1U : 0U);
        visited[i] = 0U;
        if (hot_mask[i] != 0U) {
            out->total_hot_count++;
        }
    }

    if ((max32 < ABSOLUTE_MIN_TARGET_TEMP_C) || ((int)out->total_hot_count < effective_min_pixels)) {
        out->total_hot_count = proc_to_sensor_hotcount(out->total_hot_count);
        return;
    }

    int16_t queue_x[THERMAL_PROC_PIXELS];
    int16_t queue_y[THERMAL_PROC_PIXELS];

    for (uint16_t sy = 0U; sy < THERMAL_PROC_H; sy++) {
        for (uint16_t sx = 0U; sx < THERMAL_PROC_W; sx++) {
            uint16_t start_idx = (uint16_t)((sy * THERMAL_PROC_W) + sx);
            if ((connect_mask[start_idx] == 0U) || (visited[start_idx] != 0U)) {
                continue;
            }

            int16_t head = 0;
            int16_t tail = 0;
            queue_x[tail] = (int16_t)sx;
            queue_y[tail] = (int16_t)sy;
            tail++;
            visited[start_idx] = 1U;

            uint16_t min_x = sx;
            uint16_t max_x = sx;
            uint16_t min_y = sy;
            uint16_t max_y = sy;
            uint32_t hot_count = 0U;
            float weighted_sum_x = 0.0f;
            float weighted_sum_y = 0.0f;
            float total_weight = 0.0f;
            float peak_temp = proc32[start_idx];

            while (head < tail) {
                int16_t x = queue_x[head];
                int16_t y = queue_y[head];
                head++;

                uint16_t idx = (uint16_t)((y * (int16_t)THERMAL_PROC_W) + x);
                float t = proc32[idx];

                float base = threshold;
                if (ABSOLUTE_MIN_TARGET_TEMP_C < base) {
                    base = ABSOLUTE_MIN_TARGET_TEMP_C;
                }
                float w = t - base;
                if (w < 0.0f) {
                    w = 0.0f;
                }
                if (w == 0.0f) {
                    w = 0.001f;
                }

                hot_count++;
                if ((uint16_t)x < min_x)
                    min_x = (uint16_t)x;
                if ((uint16_t)x > max_x)
                    max_x = (uint16_t)x;
                if ((uint16_t)y < min_y)
                    min_y = (uint16_t)y;
                if ((uint16_t)y > max_y)
                    max_y = (uint16_t)y;
                if (t > peak_temp)
                    peak_temp = t;

                weighted_sum_x += ((float)x) * w;
                weighted_sum_y += ((float)y) * w;
                total_weight += w;

                if (y > 0) {
                    uint16_t nidx = (uint16_t)(((y - 1) * (int16_t)THERMAL_PROC_W) + x);
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue_x[tail] = x;
                        queue_y[tail] = (int16_t)(y - 1);
                        tail++;
                    }
                }
                if (y < ((int16_t)THERMAL_PROC_H - 1)) {
                    uint16_t nidx = (uint16_t)(((y + 1) * (int16_t)THERMAL_PROC_W) + x);
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue_x[tail] = x;
                        queue_y[tail] = (int16_t)(y + 1);
                        tail++;
                    }
                }
                if (x > 0) {
                    uint16_t nidx = (uint16_t)((y * (int16_t)THERMAL_PROC_W) + (x - 1));
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue_x[tail] = (int16_t)(x - 1);
                        queue_y[tail] = y;
                        tail++;
                    }
                }
                if (x < ((int16_t)THERMAL_PROC_W - 1)) {
                    uint16_t nidx = (uint16_t)((y * (int16_t)THERMAL_PROC_W) + (x + 1));
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue_x[tail] = (int16_t)(x + 1);
                        queue_y[tail] = y;
                        tail++;
                    }
                }
            }

            if ((hot_count < (uint32_t)effective_min_pixels) || (total_weight <= 0.0f)) {
                continue;
            }

            if (out->count < THERMAL_MAX_OBJECTS) {
                                /* map 32x32 blob back to 8x8 box */
                uint16_t min_x8 = (uint16_t)((min_x * 7U) / (THERMAL_PROC_W - 1U));
                uint16_t max_x8 =
                    (uint16_t)(((max_x * 7U) + (THERMAL_PROC_W - 2U)) / (THERMAL_PROC_W - 1U));
                uint16_t min_y8 = (uint16_t)((min_y * 7U) / (THERMAL_PROC_H - 1U));
                uint16_t max_y8 =
                    (uint16_t)(((max_y * 7U) + (THERMAL_PROC_H - 2U)) / (THERMAL_PROC_H - 1U));

                /* map 32x32 blob back to 8x8 box */
                obj->valid = 1U;
                obj->min_x = (int8_t)clampu16_local(min_x8, 0U, 7U);
                obj->max_x = (int8_t)clampu16_local(max_x8, 0U, 7U);
                obj->min_y = (int8_t)clampu16_local(min_y8, 0U, 7U);
                obj->max_y = (int8_t)clampu16_local(max_y8, 0U, 7U);
                obj->centroid_x = (weighted_sum_x / total_weight) * (7.0f / (float)(THERMAL_PROC_W - 1U));
                obj->centroid_y = (weighted_sum_y / total_weight) * (7.0f / (float)(THERMAL_PROC_H - 1U));
                obj->peak_temp_c = peak_temp;
                obj->hot_count = proc_to_sensor_hotcount(hot_count);
                out->count++;
            }
        }
    }

    out->total_hot_count = proc_to_sensor_hotcount(out->total_hot_count);

    if (out->count > 1U) {
        sort_objects_by_priority(out);
    }
}

void Thermal_AnalyzeFrame8x8(const float frame_c[64], ThermalDetection *out)
{
    if ((frame_c == 0) || (out == 0)) {
        return;
    }

    ThermalObjectsResult objs;
    Thermal_DetectObjects8x8(frame_c, &objs);

    out->avg_temp_c = objs.avg_temp_c;
    out->threshold_c = objs.threshold_c;
    out->max_temp_c = objs.max_temp_c;
    out->hot_count = objs.total_hot_count;
    out->target_found = 0U;
    out->min_x = -1;
    out->max_x = -1;
    out->min_y = -1;
    out->max_y = -1;
    out->centroid_x = -1.0f;
    out->centroid_y = -1.0f;

    if (objs.count > 0U) {
        const ThermalObject *obj = &objs.objects[0];
        out->target_found = 1U;
        out->min_x = obj->min_x;
        out->max_x = obj->max_x;
        out->min_y = obj->min_y;
        out->max_y = obj->max_y;
        out->centroid_x = obj->centroid_x;
        out->centroid_y = obj->centroid_y;
    }
}

void Thermal_UpscaleNearest8x8(const float in8x8[64], float *out, uint16_t out_w, uint16_t out_h)
{
    if ((in8x8 == 0) || (out == 0) || (out_w == 0U) || (out_h == 0U)) {
        return;
    }

    for (uint16_t oy = 0U; oy < out_h; oy++) {
        uint16_t sy = (uint16_t)(((uint32_t)oy * 8U) / out_h);
        if (sy > 7U) {
            sy = 7U;
        }
        for (uint16_t ox = 0U; ox < out_w; ox++) {
            uint16_t sx = (uint16_t)(((uint32_t)ox * 8U) / out_w);
            if (sx > 7U) {
                sx = 7U;
            }
            out[(oy * out_w) + ox] = in8x8[(sy * 8U) + sx];
        }
    }
}

void Thermal_UpscaleBilinear8x8(const float in8x8[64], float *out, uint16_t out_w, uint16_t out_h)
{
    if ((in8x8 == 0) || (out == 0) || (out_w == 0U) || (out_h == 0U)) {
        return;
    }

    /* Keep API compatibility, but use the same hotspot-preserving interpolation style as detection. */
    float sum = 0.0f;
    for (uint16_t i = 0U; i < 64U; i++) {
        sum += in8x8[i];
    }
    float ambient = sum / 64.0f;

    const float sx = (out_w > 1U) ? (7.0f / (float)(out_w - 1U)) : 0.0f;
    const float sy = (out_h > 1U) ? (7.0f / (float)(out_h - 1U)) : 0.0f;

    for (uint16_t oy = 0U; oy < out_h; oy++) {
        float gy = ((float)oy) * sy;
        int32_t y0 = (int32_t)gy;
        int32_t y1 = (y0 < 7) ? (y0 + 1) : 7;
        float fy = gy - (float)y0;

        for (uint16_t ox = 0U; ox < out_w; ox++) {
            float gx = ((float)ox) * sx;
            int32_t x0 = (int32_t)gx;
            int32_t x1 = (x0 < 7) ? (x0 + 1) : 7;
            float fx = gx - (float)x0;

            float v00 = in8x8[(y0 * 8) + x0];
            float v10 = in8x8[(y0 * 8) + x1];
            float v01 = in8x8[(y1 * 8) + x0];
            float v11 = in8x8[(y1 * 8) + x1];

            float base = (v00 * (1.0f - fx) * (1.0f - fy)) +
                         (v10 * fx * (1.0f - fy)) +
                         (v01 * (1.0f - fx) * fy) +
                         (v11 * fx * fy);

            int32_t xn = (int32_t)(gx + 0.5f);
            int32_t yn = (int32_t)(gy + 0.5f);
            uint16_t sxn = clampu16_local(xn, 0U, 7U);
            uint16_t syn = clampu16_local(yn, 0U, 7U);
            float anchor = in8x8[(syn * 8U) + sxn];

            float grad_x = v10 - v00;
            float grad_y = v01 - v00;
            float edge_strength = clampf_local((grad_x * grad_x) + (grad_y * grad_y), 0.0f, 5.0f);
            float edge_mix = clampf_local(THERMAL_PROC_EDGE_GAIN * edge_strength, 0.05f, 0.75f);

            float mixed = (base * (1.0f - edge_mix)) + (anchor * edge_mix);
            if ((anchor - ambient) > THERMAL_PROC_BOOST_THRESHOLD_C) {
                mixed += THERMAL_PROC_BOOST_GAIN * (anchor - mixed);
            }

            out[(oy * out_w) + ox] = mixed;
        }
    }
}





