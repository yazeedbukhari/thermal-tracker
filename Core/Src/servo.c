/*
 * servo.c — Servo PWM control via TIM3 (CH1 = pan, CH2 = tilt)
 *
 * Initializes TIM3 at 50 Hz and maps floating-point angles (0-180 degrees)
 * to the corresponding pulse width on each timer channel.
 *
 * Owner:
 */

#include "servo.h"
