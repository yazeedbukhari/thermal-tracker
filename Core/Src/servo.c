/*
 * servo.c — Servo PWM control via TIM3 (CH1 = pan, CH2 = tilt)
 *
 * Initializes TIM3 at 50 Hz and maps floating-point angles (0-180 degrees)
 * to the corresponding pulse width on each timer channel.
 *
 * Owner:
 */

// pan uses Pin PA6 (TIM3_CH1)
// tilt uses Pin PA7 (TIM3_CH2)
 
#include "servo.h"
#include "config.h"

static float current_pan  = INIT_ANGLE;
static float current_tilt = INIT_ANGLE;

/* Maps angle [MIN_ANGLE..MAX_ANGLE] degrees to CCR (microseconds at 1 MHz tick).
 * 0° = 500 us, 90° = 1500 us, 180° = 2500 us. */
static float clamp_angle(float angle)
{
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    return angle;
}

static uint32_t angle_to_ccr(float angle)
{
    angle = clamp_angle(angle);
    return (uint32_t)(500.0f + (angle / 180.0f) * 2000.0f);
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim3_servos, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3_servos, TIM_CHANNEL_2);
    Servo_SetPan(current_pan);
    Servo_SetTilt(current_tilt);
}

void Servo_SetPan(float angle)
{
    current_pan = clamp_angle(angle);
    __HAL_TIM_SET_COMPARE(&htim3_servos, TIM_CHANNEL_1, angle_to_ccr(current_pan));
}

void Servo_SetTilt(float angle)
{
    current_tilt = clamp_angle(angle);
    __HAL_TIM_SET_COMPARE(&htim3_servos, TIM_CHANNEL_2, angle_to_ccr(current_tilt));
}

float Servo_GetPan(void)  
{ 
    return current_pan;
}
float Servo_GetTilt(void) 
{ 
    return current_tilt; 
}
