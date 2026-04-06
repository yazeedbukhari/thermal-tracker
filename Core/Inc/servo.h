#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "config.h"

#define INIT_ANGLE 90

#define MAX_ANGLE 180
#define MIN_ANGLE 0

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} servo_t;

void Servo_Init(void);

void Servo_SetPan(float angle);

void Servo_SetTilt(float angle);

float Servo_GetPan(void);

float Servo_GetTilt(void);

#endif /* INC_SERVO_H_ */
