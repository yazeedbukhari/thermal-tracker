/*
 * uart_stream.c — USART3 binary frame protocol for PC visualization
 *
 * Packs the 64x64 upscaled thermal frame, FSM state, servo angles, and
 * centroid data into a compact binary packet and transmits it over USART3
 * (routed through ST-LINK VCP) for real-time PC display.
 *
 * Owner:
 */

#include "uart_stream.h"
#include "config.h"
#include "thermal.h"
#include "servo.h"
#include "st7735_cn8_spi_test.h"
#include <string.h>
#include <stdio.h>

#define UPSCALE_W              64U
#define UPSCALE_H              64U
#define UPSCALED_STREAM_PERIOD_MS 100U
#define STREAM_OBJ_COUNT       THERMAL_MAX_OBJECTS
#define Q_TEMP_MIN_C           18.0f
#define Q_TEMP_MAX_C           35.0f

/* ── Private helpers ─────────────────────────────────────────────────────── */

static int16_t to_fixed100(float v)
{
  return (int16_t)(v * 100.0f);
}

static uint8_t quantize_u8(float t)
{
  float n = (t - Q_TEMP_MIN_C) / (Q_TEMP_MAX_C - Q_TEMP_MIN_C);
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  return (uint8_t)(n * 255.0f);
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

  HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)idx, 200);
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

  HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)idx, 200);
}

static void uart_send_um64_packet(
  uint16_t seq,
  const float *up,
  const ThermalObjectsResult *objs,
  uint8_t selected_idx,
  int16_t servo_deg_x10
)
{
  uint8_t header[20];
  uint8_t objtab[STREAM_OBJ_COUNT * 12U];
  uint16_t payload_len = (uint16_t)(UPSCALE_W * UPSCALE_H);
  static uint8_t payload[UPSCALE_W * UPSCALE_H];

  for (uint16_t i = 0; i < payload_len; i++) {
    payload[i] = quantize_u8(up[i]);
  }

  int16_t avg = to_fixed100(objs->avg_temp_c);
  int16_t thr = to_fixed100(objs->threshold_c);
  int16_t mx  = to_fixed100(objs->max_temp_c);

  header[0] = 'U'; header[1] = 'M'; header[2] = '6'; header[3] = '4';
  header[4] = (uint8_t)(seq & 0xFF);
  header[5] = (uint8_t)((seq >> 8) & 0xFF);
  header[6] = (uint8_t)UPSCALE_W;
  header[7] = (uint8_t)UPSCALE_H;
  header[8] = objs->count;
  header[9] = (objs->count > 0U) ? selected_idx : 0xFFU;
  header[10] = (uint8_t)(avg & 0xFF); header[11] = (uint8_t)((avg >> 8) & 0xFF);
  header[12] = (uint8_t)(thr & 0xFF); header[13] = (uint8_t)((thr >> 8) & 0xFF);
  header[14] = (uint8_t)(mx & 0xFF);  header[15] = (uint8_t)((mx >> 8) & 0xFF);
  header[16] = (uint8_t)(servo_deg_x10 & 0xFF); header[17] = (uint8_t)((servo_deg_x10 >> 8) & 0xFF);
  header[18] = (uint8_t)(payload_len & 0xFF); header[19] = (uint8_t)((payload_len >> 8) & 0xFF);

  for (uint8_t i = 0U; i < STREAM_OBJ_COUNT; i++) {
    const ThermalObject *obj = &objs->objects[i];
    uint8_t *p = &objtab[i * 12U];
    p[0] = obj->valid;
    p[1] = (uint8_t)obj->min_x;
    p[2] = (uint8_t)obj->max_x;
    p[3] = (uint8_t)obj->min_y;
    p[4] = (uint8_t)obj->max_y;
    {
      int16_t cx = to_fixed100(obj->centroid_x);
      int16_t cy = to_fixed100(obj->centroid_y);
      int16_t pk = to_fixed100(obj->peak_temp_c);
      p[5] = (uint8_t)(cx & 0xFF); p[6] = (uint8_t)((cx >> 8) & 0xFF);
      p[7] = (uint8_t)(cy & 0xFF); p[8] = (uint8_t)((cy >> 8) & 0xFF);
      p[9] = (uint8_t)(pk & 0xFF); p[10] = (uint8_t)((pk >> 8) & 0xFF);
    }
    p[11] = (obj->hot_count > 255U) ? 255U : (uint8_t)obj->hot_count;
  }

  HAL_UART_Transmit(&huart3, header, sizeof(header), 200);
  HAL_UART_Transmit(&huart3, objtab, sizeof(objtab), 200);
  HAL_UART_Transmit(&huart3, payload, payload_len, 400);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

/*
 * UartStream_SendFrame — upscale frame, render to TFT, send UART packet.
 * Call once per frame after ObjTracker_BuildDetection.
 */
void UartStream_SendFrame(
  const float               *frame,
  float                     *upscaled,
  const ThermalObjectsResult *objs,
  const ThermalDetection    *det,
  uint32_t                   now
)
{
#if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
  static uint32_t next_up_tx_ms = 0U;
  static uint16_t up_seq = 0U;
#endif
  static uint8_t tft_render_toggle = 0U;

  Thermal_UpscaleBilinear8x8(frame, upscaled, UPSCALE_W, UPSCALE_H);

#if ST7735_CN8_SPI_LIVE_VIEW
  tft_render_toggle ^= 1U;
  if (tft_render_toggle != 0U) {
    ST7735_CN8_RenderFrame32x32(upscaled, objs, ObjTracker_GetSelected());
  }
#endif

#if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
  if ((int32_t)(now - next_up_tx_ms) >= 0) {
    next_up_tx_ms = now + UPSCALED_STREAM_PERIOD_MS;
    uart_send_um64_packet(
      up_seq++, upscaled, objs,
      ObjTracker_GetSelected(),
      (int16_t)(Servo_GetPan() * 10.0f)
    );
  }
#elif !ST7735_CN8_SPI_LIVE_VIEW
  uart_send("BEGIN\r\n");
  uart_send_frame_csv(frame);
  uart_send_meta_csv(det, Servo_GetPan());
  uart_send("END\r\n");
#else
  (void)det;
  (void)now;
#endif
}

void uart_send(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
}