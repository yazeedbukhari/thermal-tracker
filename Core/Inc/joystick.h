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



#endif /* INC_JOYSTICK_H_ */
