/*
 * helper.h — Legacy utilities (UART print + blocking joystick ADC read)
 *
 * DEPRECATED: Contents are migrating to joystick.c (DMA-based reads) and
 * uart_stream.c (binary frame protocol). This file will be removed once
 * the new modules are functional.
 */

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
