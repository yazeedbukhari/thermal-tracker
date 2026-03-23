/*
 * joystick.c — Analog joystick input (ADC1 DMA, 2-channel scan)
 *
 * Reads X/Y axes from the joystick via ADC1 DMA continuous conversion.
 * Returns normalized values in -1.0..+1.0 with a ~10% center dead zone.
 * Replaces the old helper.c polling implementation.
 *
 * Owner: Yazeed
 */

#include "joystick.h"
