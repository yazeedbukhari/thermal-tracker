/* Declarations for joystick reading and normalization.  */
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



