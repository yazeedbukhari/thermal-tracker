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
#include "joystick.h"
#include "uart_stream.h"
#include <stdio.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();   // must come before ADC init
  MX_ADC1_Init();  // Initialize ADC1 for joystick (PA3=VRx, PC0=VRy)
  Joystick_Init(); // Start ADC1 DMA2 continuous scan

  char message[100];
  while (1)
  {
	  JoystickReading r = read_joystick_adc();
	  sprintf(message, "Vr_x: %c, Vr_y: %c\r\n",
	          r.vr_x > 0.0f ? '+' : (r.vr_x < 0.0f ? '-' : '0'),
	          r.vr_y > 0.0f ? '+' : (r.vr_y < 0.0f ? '-' : '0'));
	  print_uart(message);
	  HAL_Delay(1000);
  }
}
