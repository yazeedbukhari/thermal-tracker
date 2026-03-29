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
#include "amg8833.h"
#include "joystick.h"
#include "servo.h"
#include "uart_stream.h"
#include <stdio.h>
#include <string.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)
#define AMG8833_BRINGUP_TEST 1U

static void uart_send(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), 100);
}

static void uart_send_frame_csv(const float frame[AMG8833_PIXEL_COUNT])
{
  char line[900];
  size_t idx = 0;

  idx += (size_t)snprintf(line + idx, sizeof(line) - idx, "FRAME,");
  for (uint32_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
    int32_t c100 = (int32_t)(frame[i] * 100.0f);
    int32_t abs_c100 = (c100 < 0) ? -c100 : c100;
    int written = snprintf(
      line + idx,
      sizeof(line) - idx,
      "%s%ld.%02ld%s",
      (c100 < 0) ? "-" : "",
      (long)(abs_c100 / 100),
      (long)(abs_c100 % 100),
      (i < (AMG8833_PIXEL_COUNT - 1U)) ? "," : "\r\n"
    );

    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
      uart_send("ERR_FMT\r\n");
      return;
    }
    idx += (size_t)written;
  }

  HAL_UART_Transmit(&huart3, (uint8_t*)line, (uint16_t)idx, 200);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
#if AMG8833_BRINGUP_TEST
  MX_I2C1_Init();
#else
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();   // must come before ADC init
  MX_ADC1_Init();  // Initialize ADC1 for joystick (PA3=VRx, PC0=VRy)
  MX_TIM3_Init();  // Initialize TIM3 PWM for servos (PA6=pan, PA7=tilt)
  Joystick_Init(); // Start ADC1 DMA2 continuous scan
  Servo_Init();    // Start TIM3 PWM, center both servos at 90°
#endif

#if AMG8833_BRINGUP_TEST
  uint8_t amg_addr = 0;
  float frame[AMG8833_PIXEL_COUNT];
  HAL_Delay(200);

  uart_send("AMG8833 bring-up test\r\n");
  if (AMG8833_Probe(&hi2c1, &amg_addr) != HAL_OK) {
    uart_send("ERR_PROBE (check wiring/address)\r\n");
    while (1) {
      HAL_Delay(500);
    }
  }

  if (AMG8833_Init(&hi2c1, amg_addr) != HAL_OK) {
    uart_send("ERR_INIT\r\n");
    while (1) {
      HAL_Delay(500);
    }
  }

  char addr_msg[32];
  (void)snprintf(addr_msg, sizeof(addr_msg), "READY addr=0x%02X\r\n", amg_addr);
  uart_send(addr_msg);
#endif

  while (1)
  {
#if AMG8833_BRINGUP_TEST
    if (AMG8833_ReadFrameCelsius(&hi2c1, amg_addr, frame) != HAL_OK) {
      uart_send("ERR_READ\r\n");
      HAL_Delay(250);
      continue;
    }

    uart_send("BEGIN\r\n");
    uart_send_frame_csv(frame);
    uart_send("END\r\n");
    HAL_Delay(500);
#else
		  JoystickReading r = read_joystick_adc();
	    
    Servo_SetPan(Servo_GetPan() + r.vr_x);
    Servo_SetTilt(Servo_GetTilt() + r.vr_y);
		  HAL_Delay(1);
#endif
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PIN_SW)
    {
#if AMG8833_BRINGUP_TEST
        uart_send("SW\r\n");
#else
        char msg[40];
        int pan  = (int)Servo_GetPan();
        int tilt = (int)Servo_GetTilt();
        int len = sprintf(msg, "pan: %d, tilt: %d\r\n", pan, tilt);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
#endif
    }
}
