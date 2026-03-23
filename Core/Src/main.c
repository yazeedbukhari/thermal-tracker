/*
 * main.c — Application entry point and super loop
 *
 * Initializes all peripherals, then runs the 10 Hz frame-synchronous pipeline:
 *   DMA frame ready → swap buffers → interpolate → centroid → [Kalman] →
 *   FSM → [PID] → servo update → laser control → UART stream
 *
 * ISR callbacks for I2C DMA complete and EXTI (USER button) live here or
 * in stm32f4xx_it.c. The loop spins on AMG8833_FrameReady() between frames.
 */

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
