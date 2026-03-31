/*
 * joystick.h — Analog joystick input (ADC1 DMA, 2-channel scan)
 *
 * Reads X/Y axes from the joystick via ADC1 (PA0 = IN0, PA1 = IN1) using
 * DMA continuous conversion. Returns normalized values in -1.0..+1.0 with
 * a ~10% center dead zone applied. Replaces the old helper.h polling reads.
 *
 * Public API:  Joystick_Init, Joystick_ReadX, Joystick_ReadY
 *
 * Owner: Yazeed
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

typedef struct {
    float vr_x; /* normalized -1.0..+1.0, dead zone applied */
    float vr_y;
} JoystickReading;

void Joystick_Init(void);
void Joystick_DeInit(void);
JoystickReading read_joystick_adc(void);

#endif /* INC_JOYSTICK_H_ */
