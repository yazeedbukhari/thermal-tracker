/*
 * servo.h — Servo PWM control via TIM3 (CH1 = pan, CH2 = tilt)
 *
 * Initializes TIM3 at 50 Hz and provides angle-to-CCR mapping functions.
 * Accepts a floating-point angle in degrees (0-180) and sets the
 * corresponding pulse width on the appropriate timer channel.
 *
 * Public API:  Servo_Init, Servo_SetPan, Servo_SetTilt
 *
 * Owner:
 */

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
