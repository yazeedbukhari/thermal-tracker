#include "helper.h"
#include "config.h"

void print_uart(char * msg) {
	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

JoystickReading read_joystick_adc(void)
{
    JoystickReading reading;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    reading.vr_x = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    reading.vr_y = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return reading;
}
