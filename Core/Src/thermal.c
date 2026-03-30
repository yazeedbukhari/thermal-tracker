/*
 * thermal.c - Thermal frame analysis (dynamic threshold + bbox + centroid)
 *
 * Implements the same dynamic-threshold detection logic used in the Arduino
 * bounding box prototype, adapted for STM32 project modules.
 *
 * Owner: Ali
 */

#include "thermal.h"

#define TEMP_OFFSET_C               4.1f
#define ABSOLUTE_TARGET_CLAMP_C     32.0f
#define ABSOLUTE_MIN_TARGET_TEMP_C  25.2f
#define MIN_HOT_PIXELS              5
#define CONNECT_OFFSET_C            0.9f

static void clear_objects(ThermalObjectsResult *out)
{
    out->avg_temp_c = 0.0f;
    out->threshold_c = 0.0f;
    out->max_temp_c = -1000.0f;
    out->total_hot_count = 0U;
    out->count = 0U;
    for (uint32_t i = 0; i < THERMAL_MAX_OBJECTS; i++) {
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
    for (uint32_t i = 0; i + 1U < out->count; i++) {
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

void Thermal_DetectObjects8x8(const float frame_c[64], ThermalObjectsResult *out)
{
    if ((frame_c == 0) || (out == 0)) {
        return;
    }

    clear_objects(out);

    float sum = 0.0f;
    float max_temp = -1000.0f;
    for (int i = 0; i < 64; i++) {
        float t = frame_c[i];
        sum += t;
        if (t > max_temp) {
            max_temp = t;
        }
    }

    float avg_temp = sum / 64.0f;
    float threshold = avg_temp + TEMP_OFFSET_C;
    if (threshold > ABSOLUTE_TARGET_CLAMP_C) {
        threshold = ABSOLUTE_TARGET_CLAMP_C;
    }

    int effective_min_pixels = MIN_HOT_PIXELS;
    if (max_temp > (ABSOLUTE_TARGET_CLAMP_C + 2.0f)) {
        effective_min_pixels = 3;
    }

    out->avg_temp_c = avg_temp;
    out->threshold_c = threshold;
    out->max_temp_c = max_temp;

    uint8_t hot_mask[64];
    uint8_t connect_mask[64];
    uint8_t visited[64];
    float connect_threshold = threshold + CONNECT_OFFSET_C;
    float connect_absolute = ABSOLUTE_MIN_TARGET_TEMP_C + CONNECT_OFFSET_C;
    for (int i = 0; i < 64; i++) {
        float t = frame_c[i];
        hot_mask[i] = (uint8_t)(((t >= threshold) || (t >= ABSOLUTE_MIN_TARGET_TEMP_C)) ? 1U : 0U);
        connect_mask[i] = (uint8_t)(((t >= connect_threshold) || (t >= connect_absolute)) ? 1U : 0U);
        visited[i] = 0U;
        if (hot_mask[i] != 0U) {
            out->total_hot_count++;
        }
    }

    if ((max_temp < ABSOLUTE_MIN_TARGET_TEMP_C) || ((int)out->total_hot_count < effective_min_pixels)) {
        return;
    }

    int queue[64];
    for (int sy = 0; sy < 8; sy++) {
        for (int sx = 0; sx < 8; sx++) {
            int start_idx = (sy * 8) + sx;
            if ((connect_mask[start_idx] == 0U) || (visited[start_idx] != 0U)) {
                continue;
            }

            int head = 0;
            int tail = 0;
            queue[tail++] = start_idx;
            visited[start_idx] = 1U;

            int min_x = sx;
            int max_x = sx;
            int min_y = sy;
            int max_y = sy;
            int hot_count = 0;
            float weighted_sum_x = 0.0f;
            float weighted_sum_y = 0.0f;
            float total_weight = 0.0f;
            float peak_temp = frame_c[start_idx];

            while (head < tail) {
                int idx = queue[head++];
                int y = idx / 8;
                int x = idx % 8;
                float t = frame_c[idx];
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
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (t > peak_temp) peak_temp = t;

                weighted_sum_x += ((float)x) * w;
                weighted_sum_y += ((float)y) * w;
                total_weight += w;

                /* 4-neighbor connectivity avoids diagonal bridging between objects. */
                if (y > 0) {
                    int nidx = ((y - 1) * 8) + x;
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue[tail++] = nidx;
                    }
                }
                if (y < 7) {
                    int nidx = ((y + 1) * 8) + x;
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue[tail++] = nidx;
                    }
                }
                if (x > 0) {
                    int nidx = (y * 8) + (x - 1);
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue[tail++] = nidx;
                    }
                }
                if (x < 7) {
                    int nidx = (y * 8) + (x + 1);
                    if ((connect_mask[nidx] != 0U) && (visited[nidx] == 0U)) {
                        visited[nidx] = 1U;
                        queue[tail++] = nidx;
                    }
                }
            }

            if ((hot_count < effective_min_pixels) || (total_weight <= 0.0f)) {
                continue;
            }

            if (out->count < THERMAL_MAX_OBJECTS) {
                ThermalObject *obj = &out->objects[out->count];
                obj->valid = 1U;
                obj->min_x = (int8_t)min_x;
                obj->max_x = (int8_t)max_x;
                obj->min_y = (int8_t)min_y;
                obj->max_y = (int8_t)max_y;
                obj->centroid_x = weighted_sum_x / total_weight;
                obj->centroid_y = weighted_sum_y / total_weight;
                obj->peak_temp_c = peak_temp;
                obj->hot_count = (uint16_t)hot_count;
                out->count++;
            }
        }
    }

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

void Thermal_UpscaleBilinear8x8(
    const float in8x8[64],
    float *out,
    uint16_t out_w,
    uint16_t out_h
)
{
    if ((in8x8 == 0) || (out == 0) || (out_w == 0U) || (out_h == 0U)) {
        return;
    }

    const float sx = (out_w > 1U) ? (7.0f / (float)(out_w - 1U)) : 0.0f;
    const float sy = (out_h > 1U) ? (7.0f / (float)(out_h - 1U)) : 0.0f;

    for (uint16_t oy = 0; oy < out_h; oy++) {
        float gy = ((float)oy) * sy;
        int y0 = (int)gy;
        int y1 = (y0 < 7) ? (y0 + 1) : 7;
        float dy = gy - (float)y0;

        for (uint16_t ox = 0; ox < out_w; ox++) {
            float gx = ((float)ox) * sx;
            int x0 = (int)gx;
            int x1 = (x0 < 7) ? (x0 + 1) : 7;
            float dx = gx - (float)x0;

            float v00 = in8x8[(y0 * 8) + x0];
            float v10 = in8x8[(y0 * 8) + x1];
            float v01 = in8x8[(y1 * 8) + x0];
            float v11 = in8x8[(y1 * 8) + x1];

            out[(oy * out_w) + ox] =
                (v00 * (1.0f - dx) * (1.0f - dy)) +
                (v10 * dx * (1.0f - dy)) +
                (v01 * (1.0f - dx) * dy) +
                (v11 * dx * dy);
        }
    }
}
