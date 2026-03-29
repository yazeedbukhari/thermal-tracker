/*
 * thermal.h - Thermal frame analysis (dynamic threshold + bbox + centroid)
 *
 * Pure math module with no hardware dependencies. Takes one raw 8x8 thermal
 * frame (64 values, row-major), computes dynamic-threshold hot-pixel detection,
 * and outputs bounding box + weighted centroid metadata.
 *
 * Public API:  Thermal_AnalyzeFrame8x8
 *
 * Owner:
 */

#ifndef INC_THERMAL_H_
#define INC_THERMAL_H_

#include <stdint.h>

#define THERMAL_MAX_OBJECTS 4U

typedef struct {
    uint8_t target_found;
    int8_t min_x;
    int8_t max_x;
    int8_t min_y;
    int8_t max_y;
    float centroid_x;
    float centroid_y;
    float avg_temp_c;
    float threshold_c;
    float max_temp_c;
    uint16_t hot_count;
} ThermalDetection;

typedef struct {
    uint8_t valid;
    int8_t min_x;
    int8_t max_x;
    int8_t min_y;
    int8_t max_y;
    float centroid_x;
    float centroid_y;
    float peak_temp_c;
    uint16_t hot_count;
} ThermalObject;

typedef struct {
    float avg_temp_c;
    float threshold_c;
    float max_temp_c;
    uint16_t total_hot_count;
    uint8_t count;
    ThermalObject objects[THERMAL_MAX_OBJECTS];
} ThermalObjectsResult;

void Thermal_DetectObjects8x8(const float frame_c[64], ThermalObjectsResult *out);

void Thermal_AnalyzeFrame8x8(const float frame_c[64], ThermalDetection *out);

/* Bilinear upscale from 8x8 frame to arbitrary output resolution. */
void Thermal_UpscaleBilinear8x8(
    const float in8x8[64],
    float *out,
    uint16_t out_w,
    uint16_t out_h
);

#endif /* INC_THERMAL_H_ */
