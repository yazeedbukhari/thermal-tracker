/*
 * thermal.c — Thermal image processing (bilinear interpolation + centroid)
 *
 * Pure math module with no hardware dependencies. Takes a raw 8x8 thermal
 * frame, upscales it to 64x64 via bilinear interpolation, then extracts a
 * weighted sub-pixel centroid of all pixels above a temperature threshold.
 *
 * Owner: Ali
 */

#include "thermal.h"
