/*
 * tracking.h - Thermal centroid to pan/tilt servo tracker
 *
 * Uses ThermalDetection centroid (8x8 sensor coordinates) to command
 * pan/tilt servos with a lightweight proportional controller.
 */

#ifndef INC_TRACKING_H_
#define INC_TRACKING_H_

#include <stdint.h>
#include "thermal.h"

void Tracking_Init(void);
void Tracking_Enable(uint8_t enable);
void Tracking_UpdateFromDetection(const ThermalDetection *det);

#endif /* INC_TRACKING_H_ */

