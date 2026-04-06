/* Declarations for converting detections into servo tracking commands.  */
#ifndef INC_TRACKING_H_
#define INC_TRACKING_H_

#include "thermal.h"
#include <stdint.h>

void Tracking_Init(void);
void Tracking_Enable(uint8_t enable);
void Tracking_ResetSearchTimer(void);
void Tracking_UpdateFromDetection(const ThermalDetection *det);

#endif /* INC_TRACKING_H_ */



