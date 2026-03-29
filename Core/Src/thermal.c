/*
 * thermal.c - Thermal frame analysis (dynamic threshold + bbox + centroid)
 *
 * Implements the same dynamic-threshold detection logic used in the Arduino
 * bounding box prototype, adapted for STM32 project modules.
 *
 * Owner: Ali
 */

#include "thermal.h"

#define TEMP_OFFSET_C               3.0f
#define ABSOLUTE_TARGET_CLAMP_C     28.0f
#define ABSOLUTE_MIN_TARGET_TEMP_C  24.5f
#define MIN_HOT_PIXELS              5

void Thermal_AnalyzeFrame8x8(const float frame_c[64], ThermalDetection *out)
{
    if ((frame_c == 0) || (out == 0)) {
        return;
    }

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

    int min_x = 7;
    int max_x = 0;
    int min_y = 7;
    int max_y = 0;
    int hot_count = 0;

    float weighted_sum_x = 0.0f;
    float weighted_sum_y = 0.0f;
    float total_weight = 0.0f;

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            float t = frame_c[(y * 8) + x];
            int is_hot = (t >= threshold) || (t >= ABSOLUTE_MIN_TARGET_TEMP_C);

            if (is_hot) {
                hot_count++;

                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;

                float base = threshold;
                if (ABSOLUTE_MIN_TARGET_TEMP_C < base) {
                    base = ABSOLUTE_MIN_TARGET_TEMP_C;
                }

                float weight = t - base;
                if (weight < 0.0f) {
                    weight = 0.0f;
                }

                weighted_sum_x += ((float)x) * weight;
                weighted_sum_y += ((float)y) * weight;
                total_weight += weight;
            }
        }
    }

    out->target_found = 1U;
    out->centroid_x = -1.0f;
    out->centroid_y = -1.0f;
    out->avg_temp_c = avg_temp;
    out->threshold_c = threshold;
    out->max_temp_c = max_temp;
    out->hot_count = (uint16_t)hot_count;

    if ((max_temp < ABSOLUTE_MIN_TARGET_TEMP_C) ||
        (hot_count < effective_min_pixels) ||
        (total_weight <= 0.0f)) {
        out->target_found = 0U;
        out->min_x = -1;
        out->max_x = -1;
        out->min_y = -1;
        out->max_y = -1;
    } else {
        out->min_x = (int8_t)min_x;
        out->max_x = (int8_t)max_x;
        out->min_y = (int8_t)min_y;
        out->max_y = (int8_t)max_y;
        out->centroid_x = weighted_sum_x / total_weight;
        out->centroid_y = weighted_sum_y / total_weight;
    }
}
