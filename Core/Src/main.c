/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "config.h"
#include "helper.h"
#include <stdio.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init(); // Initializes gpio for A2
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init(); // Initialize ADC for A0, A1

  char message[100];
  while (1)
  {
	  JoystickReading joy = read_joystick_adc();

	  // poll a2 - make this an interrupt for firing
	  GPIO_PinState sw = HAL_GPIO_ReadPin(GPIOC, PIN_SW);

	  sprintf(message, "VRx: %lu, VRy: %lu, SW: %i\r\n", joy.vr_x, joy.vr_y, sw);
	  print_uart(message);
	  HAL_Delay(1000);
  }
}
