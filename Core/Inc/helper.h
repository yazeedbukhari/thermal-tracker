#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "main.h"

void print_uart(char * msg);

typedef struct {
    uint32_t vr_x;
    uint32_t vr_y;
} JoystickReading;

JoystickReading read_joystick_adc(void);

#endif /* INC_HELPER_H_ */
