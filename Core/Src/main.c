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
#include "thermal.h"
#include "tracking.h"
#include "joystick.h"
#include "servo.h"
#include "uart_stream.h"
#include "st7735_cn8_spi_test.h"
#include "object_tracker.h"
#include <stdio.h>
#include <string.h>

static volatile uint32_t g_last_b1_ms = 0U;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();

#if ST7735_CN8_SPI_BOX_TEST || ST7735_CN8_SPI_LIVE_VIEW
  ST7735_CN8_Test_Init();
#endif

#if ST7735_CN8_SPI_BOX_TEST
  ST7735_CN8_Test_DrawBox();
  uart_send("ST7735 CN8 SPI box test drawn\r\n");
  while (1) { HAL_Delay(1000); }
#endif

#if AMG8833_BRINGUP_TEST
  MX_I2C1_Init();
#if THERMAL_SERVO_TRACK_TEST
  MX_TIM3_Init();
  Servo_Init();
  Tracking_Init();
#endif
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
  static float upscaled[UPSCALE_W * UPSCALE_H];
  ThermalObjectsResult objs;
  ThermalDetection det;
  uint32_t next_frame_ms = HAL_GetTick();
  ObjTracker_Init();

  uart_send("AMG8833 bring-up test\r\n");
  if (AMG8833_Probe(&hi2c1, &amg_addr) != HAL_OK) {
    uart_send("ERR_PROBE (check wiring/address)\r\n");
    while (1) { HAL_Delay(100); }
  }
  if (AMG8833_Init(&hi2c1, amg_addr) != HAL_OK) {
    uart_send("ERR_INIT\r\n");
    while (1) { HAL_Delay(100); }
  }
  char addr_msg[32];
  (void)snprintf(addr_msg, sizeof(addr_msg), "READY addr=0x%02X\r\n", amg_addr);
  uart_send(addr_msg);
#endif

  while (1)
  {
#if AMG8833_BRINGUP_TEST
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame_ms) < 0) continue;
    next_frame_ms = now + AMG_FRAME_PERIOD_MS;

    if (AMG8833_ReadFrameCelsius(&hi2c1, amg_addr, frame) != HAL_OK) {
      uart_send("ERR_READ\r\n");
      continue;
    }

    Thermal_DetectObjects8x8(frame, &objs);
    ObjTracker_Associate(&objs, now);
    ObjTracker_Update(&objs, now);
    ObjTracker_BuildDetection(&det, &objs, now);

#if THERMAL_SERVO_TRACK_TEST
    Tracking_UpdateFromDetection(&det);
#endif
    UartStream_SendFrame(frame, upscaled, &objs, &det, now);

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
    if (GPIO_Pin == USER_Btn_Pin)
    {
#if AMG8833_BRINGUP_TEST
        uint32_t now = HAL_GetTick();
        if ((now - g_last_b1_ms) > 180U) {
          g_last_b1_ms = now;
          ObjTracker_RequestNext();
        }
#endif
        return;
    }

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
