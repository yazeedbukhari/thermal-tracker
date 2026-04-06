/* Shared peripheral handle declarations and init function prototypes. */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma2_joystick;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi3_tft;
extern TIM_HandleTypeDef htim3_servos;

void SystemClock_Config(void);
void MX_USART3_UART_Init(void);
void MX_I2C1_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_SPI3_Init(void);
void MX_ADC1_Init(void);
void MX_TIM3_Init(void);

#endif /* INC_CONFIG_H_ */




