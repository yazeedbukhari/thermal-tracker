#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

extern ADC_HandleTypeDef hadc1;

void SystemClock_Config(void);
void MX_USART3_UART_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);

#endif /* INC_CONFIG_H_ */
