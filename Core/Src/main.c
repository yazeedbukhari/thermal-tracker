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
#include "joystick.h"
#include "servo.h"
#include "uart_stream.h"
#include <stdio.h>
#include <string.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)
#define AMG8833_BRINGUP_TEST 1U
#define AMG_FRAME_PERIOD_MS 100U  /* AMG8833 configured at 10 FPS */

static void uart_send(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), 100);
}

static int append_fixed(char *dst, size_t cap, size_t *idx, float value, int decimals)
{
  int32_t scale = (decimals == 1) ? 10 : 100;
  int32_t scaled = (int32_t)(value * (float)scale);
  int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;
  int written;

  if (decimals == 1) {
    written = snprintf(
      dst + *idx,
      cap - *idx,
      "%s%ld.%01ld",
      (scaled < 0) ? "-" : "",
      (long)(abs_scaled / 10),
      (long)(abs_scaled % 10)
    );
  } else {
    written = snprintf(
      dst + *idx,
      cap - *idx,
      "%s%ld.%02ld",
      (scaled < 0) ? "-" : "",
      (long)(abs_scaled / 100),
      (long)(abs_scaled % 100)
    );
  }

  if ((written <= 0) || ((size_t)written >= (cap - *idx))) {
    return 0;
  }

  *idx += (size_t)written;
  return 1;
}

static void uart_send_frame_csv(const float frame[AMG8833_PIXEL_COUNT])
{
  char line[900];
  size_t idx = 0;

  idx += (size_t)snprintf(line + idx, sizeof(line) - idx, "FRAME,");
  for (uint32_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
    if (!append_fixed(line, sizeof(line), &idx, frame[i], 2)) {
      uart_send("ERR_FMT\r\n");
      return;
    }

    if (i < (AMG8833_PIXEL_COUNT - 1U)) {
      int written = snprintf(line + idx, sizeof(line) - idx, ",");
      if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
        uart_send("ERR_FMT\r\n");
        return;
      }
      idx += (size_t)written;
    } else {
      int written = snprintf(line + idx, sizeof(line) - idx, "\r\n");
      if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
        uart_send("ERR_FMT\r\n");
        return;
      }
      idx += (size_t)written;
    }
  }

  HAL_UART_Transmit(&huart3, (uint8_t*)line, (uint16_t)idx, 200);
}

static void uart_send_meta_csv(const ThermalDetection *det, float servo_angle_deg)
{
  char line[240];
  size_t idx = 0;

  int written = snprintf(
    line + idx,
    sizeof(line) - idx,
    "META,%u,%d,%d,%d,%d,",
    (unsigned)det->target_found,
    (int)det->min_x,
    (int)det->max_x,
    (int)det->min_y,
    (int)det->max_y
  );
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
    uart_send("ERR_FMT\r\n");
    return;
  }
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, det->centroid_x, 2)) return;
  written = snprintf(line + idx, sizeof(line) - idx, ",");
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, det->centroid_y, 2)) return;
  written = snprintf(line + idx, sizeof(line) - idx, ",");
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, det->avg_temp_c, 2)) return;
  written = snprintf(line + idx, sizeof(line) - idx, ",");
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, det->threshold_c, 2)) return;
  written = snprintf(line + idx, sizeof(line) - idx, ",");
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, det->max_temp_c, 2)) return;
  written = snprintf(line + idx, sizeof(line) - idx, ",%u,", (unsigned)det->hot_count);
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

  if (!append_fixed(line, sizeof(line), &idx, servo_angle_deg, 1)) return;
  written = snprintf(line + idx, sizeof(line) - idx, "\r\n");
  if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) return;
  idx += (size_t)written;

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
  ThermalDetection det;
  const float servo_angle_placeholder = 90.0f;
  uint32_t next_frame_ms = HAL_GetTick();

  uart_send("AMG8833 bring-up test\r\n");
  if (AMG8833_Probe(&hi2c1, &amg_addr) != HAL_OK) {
    uart_send("ERR_PROBE (check wiring/address)\r\n");
    while (1) {
      HAL_Delay(100);
    }
  }

  if (AMG8833_Init(&hi2c1, amg_addr) != HAL_OK) {
    uart_send("ERR_INIT\r\n");
    while (1) {
      HAL_Delay(100);
    }
  }

  char addr_msg[32];
  (void)snprintf(addr_msg, sizeof(addr_msg), "READY addr=0x%02X\r\n", amg_addr);
  uart_send(addr_msg);
#endif

  while (1)
  {
#if AMG8833_BRINGUP_TEST
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame_ms) < 0) {
      continue;
    }
    next_frame_ms = now + AMG_FRAME_PERIOD_MS;

    if (AMG8833_ReadFrameCelsius(&hi2c1, amg_addr, frame) != HAL_OK) {
      uart_send("ERR_READ\r\n");
      continue;
    }

    uart_send("BEGIN\r\n");
    uart_send_frame_csv(frame);
    Thermal_AnalyzeFrame8x8(frame, &det);
    uart_send_meta_csv(&det, servo_angle_placeholder);
    uart_send("END\r\n");
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
