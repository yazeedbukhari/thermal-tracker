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
#include <stdio.h>
#include <string.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)
#define AMG8833_BRINGUP_TEST 1U
#define AMG_FRAME_PERIOD_MS 100U  /* AMG8833 configured at 10 FPS */
#define STREAM_MULTI_OBJECT_BINARY 1U
#define UPSCALE_W 64U
#define UPSCALE_H 64U
#define UPSCALED_STREAM_PERIOD_MS 100U  /* send each sensor frame (~10 FPS) */
#define Q_TEMP_MIN_C 18.0f
#define Q_TEMP_MAX_C 35.0f
#define THERMAL_SERVO_TRACK_TEST 1U
#define STREAM_OBJ_COUNT THERMAL_MAX_OBJECTS
#define ST7735_CN8_SPI_BOX_TEST 0U
#define ST7735_CN8_SPI_LIVE_VIEW 1U
#define OBJ_ID_NONE 0xFFU
#define OBJ_ASSOC_MAX_DIST 2.6f
#define OBJ_TRACK_STALE_MS 1500U
#define SEEK_OTHER_MIN_MANHATTAN 1.4f
#define SELECT_FALLBACK_LAG_MS 1200U
#define SEEK_PRE_SCAN_WAIT_MS 2200U
#define SEEK_ACCEPT_VISIBLE_MS 600U

static volatile uint8_t g_next_object_request = 0U;
static uint8_t g_selected_object = 0U;
static uint8_t g_selected_object_id = 0U;
static volatile uint8_t g_seek_other_object = 0U;
static volatile uint32_t g_last_b1_ms = 0U;
static uint8_t g_obj_ids[THERMAL_MAX_OBJECTS] = {OBJ_ID_NONE, OBJ_ID_NONE, OBJ_ID_NONE, OBJ_ID_NONE};
static uint8_t g_seek_has_ref = 0U;
static float g_seek_ref_cx = 0.0f;
static float g_seek_ref_cy = 0.0f;
static uint32_t g_selected_missing_since_ms = 0U;
static uint32_t g_seek_enter_ms = 0U;
static uint32_t g_seek_visible_since_ms = 0U;

typedef struct {
  uint8_t active;
  float cx;
  float cy;
  uint32_t last_seen_ms;
} ObjTrack;

static ObjTrack g_tracks[THERMAL_MAX_OBJECTS];

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

static uint8_t track_find_free(uint32_t now_ms, const uint8_t used[THERMAL_MAX_OBJECTS])
{
  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if ((used[t] == 0U) && ((g_tracks[t].active == 0U) || ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS))) {
      return t;
    }
  }
  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if (used[t] == 0U) return t;
  }
  return 0U;
}

static void associate_object_ids(const ThermalObjectsResult *objs, uint32_t now_ms)
{
  uint8_t used_track[THERMAL_MAX_OBJECTS] = {0U};

  for (uint8_t i = 0U; i < THERMAL_MAX_OBJECTS; i++) {
    g_obj_ids[i] = OBJ_ID_NONE;
  }

  for (uint8_t i = 0U; i < objs->count; i++) {
    const ThermalObject *obj = &objs->objects[i];
    if (obj->valid == 0U) continue;

    float best = 9999.0f;
    uint8_t best_id = OBJ_ID_NONE;
    for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
      if ((g_tracks[t].active == 0U) || (used_track[t] != 0U)) continue;
      if ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS) continue;

      float d = absf_local(obj->centroid_x - g_tracks[t].cx) + absf_local(obj->centroid_y - g_tracks[t].cy);
      if ((d < best) && (d <= OBJ_ASSOC_MAX_DIST)) {
        best = d;
        best_id = t;
      }
    }

    if (best_id != OBJ_ID_NONE) {
      g_obj_ids[i] = best_id;
      used_track[best_id] = 1U;
      g_tracks[best_id].active = 1U;
      g_tracks[best_id].cx = obj->centroid_x;
      g_tracks[best_id].cy = obj->centroid_y;
      g_tracks[best_id].last_seen_ms = now_ms;
    }
  }

  for (uint8_t i = 0U; i < objs->count; i++) {
    const ThermalObject *obj = &objs->objects[i];
    if ((obj->valid == 0U) || (g_obj_ids[i] != OBJ_ID_NONE)) continue;

    uint8_t id = track_find_free(now_ms, used_track);
    g_obj_ids[i] = id;
    used_track[id] = 1U;
    g_tracks[id].active = 1U;
    g_tracks[id].cx = obj->centroid_x;
    g_tracks[id].cy = obj->centroid_y;
    g_tracks[id].last_seen_ms = now_ms;
  }

  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if ((g_tracks[t].active != 0U) && ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS)) {
      g_tracks[t].active = 0U;
    }
  }
}

static int8_t find_obj_index_by_id(const ThermalObjectsResult *objs, uint8_t id)
{
  if (id == OBJ_ID_NONE) return -1;
  for (uint8_t i = 0U; i < objs->count; i++) {
    if ((objs->objects[i].valid != 0U) && (g_obj_ids[i] == id)) return (int8_t)i;
  }
  return -1;
}

static uint8_t pick_next_visible_id(const ThermalObjectsResult *objs, uint8_t current_id)
{
  uint8_t ids[THERMAL_MAX_OBJECTS];
  uint8_t n = 0U;
  for (uint8_t i = 0U; i < objs->count; i++) {
    if (objs->objects[i].valid == 0U) continue;
    ids[n++] = g_obj_ids[i];
  }
  if (n == 0U) return OBJ_ID_NONE;
  if (n == 1U) return ids[0];

  for (uint8_t i = 0U; i < n; i++) {
    if (ids[i] == current_id) {
      return ids[(uint8_t)((i + 1U) % n)];
    }
  }
  return ids[0];
}

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

static void detection_from_object(
  ThermalDetection *det,
  const ThermalObjectsResult *objs,
  uint8_t selected_idx
)
{
  det->target_found = 0U;
  det->min_x = -1;
  det->max_x = -1;
  det->min_y = -1;
  det->max_y = -1;
  det->centroid_x = -1.0f;
  det->centroid_y = -1.0f;
  det->avg_temp_c = objs->avg_temp_c;
  det->threshold_c = objs->threshold_c;
  det->max_temp_c = objs->max_temp_c;
  det->hot_count = objs->total_hot_count;

  if (objs->count == 0U) {
    return;
  }

  if (selected_idx >= objs->count) {
    selected_idx = 0U;
  }

  const ThermalObject *obj = &objs->objects[selected_idx];
  det->target_found = obj->valid;
  det->min_x = obj->min_x;
  det->max_x = obj->max_x;
  det->min_y = obj->min_y;
  det->max_y = obj->max_y;
  det->centroid_x = obj->centroid_x;
  det->centroid_y = obj->centroid_y;
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
  int16_t mx = to_fixed100(objs->max_temp_c);

  header[0] = 'U'; header[1] = 'M'; header[2] = '6'; header[3] = '4';
  header[4] = (uint8_t)(seq & 0xFF);
  header[5] = (uint8_t)((seq >> 8) & 0xFF);
  header[6] = (uint8_t)UPSCALE_W;
  header[7] = (uint8_t)UPSCALE_H;
  header[8] = objs->count;
  header[9] = (objs->count > 0U) ? selected_idx : 0xFFU;
  header[10] = (uint8_t)(avg & 0xFF); header[11] = (uint8_t)((avg >> 8) & 0xFF);
  header[12] = (uint8_t)(thr & 0xFF); header[13] = (uint8_t)((thr >> 8) & 0xFF);
  header[14] = (uint8_t)(mx & 0xFF); header[15] = (uint8_t)((mx >> 8) & 0xFF);
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

  while (1) {
    HAL_Delay(1000);
  }
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
  uint8_t tft_render_toggle = 0U;
  g_selected_object_id = OBJ_ID_NONE;
#if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
  uint32_t next_up_tx_ms = HAL_GetTick();
  uint16_t up_seq = 0;
#endif

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

    Thermal_DetectObjects8x8(frame, &objs);
    associate_object_ids(&objs, now);
    g_selected_object = 0xFFU;

    if (g_next_object_request != 0U) {
      g_next_object_request = 0U;

      if (objs.count > 1U) {
        /* Multiple objects now: switch to next visible ID. */
        g_selected_object_id = pick_next_visible_id(&objs, g_selected_object_id);
        g_seek_other_object = 0U;
        g_seek_has_ref = 0U;
        g_selected_missing_since_ms = 0U;
        g_seek_enter_ms = 0U;
        g_seek_visible_since_ms = 0U;
      } else {
        /* Single/no object: toggle seek-other mode.
         * In seek mode, tracking will scan until another object appears.
         * Pressing button again exits seek mode. */
        if (g_seek_other_object == 0U) {
          int8_t current_idx = find_obj_index_by_id(&objs, g_selected_object_id);
          g_seek_other_object = 1U;
          g_seek_has_ref = 0U;
          if ((current_idx >= 0) && (objs.objects[current_idx].valid != 0U)) {
            g_seek_ref_cx = objs.objects[current_idx].centroid_x;
            g_seek_ref_cy = objs.objects[current_idx].centroid_y;
            g_seek_has_ref = 1U;
          }
          g_seek_enter_ms = now;
          g_seek_visible_since_ms = 0U;
        } else {
          g_seek_other_object = 0U;
          g_seek_has_ref = 0U;
          g_selected_missing_since_ms = 0U;
          g_seek_enter_ms = 0U;
          g_seek_visible_since_ms = 0U;
        }
      }
    }

    if ((g_seek_other_object != 0U) && (objs.count > 0U)) {
      uint8_t switched = 0U;
      for (uint8_t i = 0U; i < objs.count; i++) {
        if (objs.objects[i].valid == 0U) continue;

        uint8_t id_diff = (g_obj_ids[i] != g_selected_object_id) ? 1U : 0U;
        uint8_t far_enough = 0U;
        if (g_seek_has_ref != 0U) {
          float md = absf_local(objs.objects[i].centroid_x - g_seek_ref_cx) +
                     absf_local(objs.objects[i].centroid_y - g_seek_ref_cy);
          if (md >= SEEK_OTHER_MIN_MANHATTAN) {
            far_enough = 1U;
          }
        }

        if ((id_diff != 0U) || (far_enough != 0U)) {
          g_selected_object_id = g_obj_ids[i];
          g_seek_other_object = 0U;
          g_seek_has_ref = 0U;
          g_selected_missing_since_ms = 0U;
          g_seek_enter_ms = 0U;
          g_seek_visible_since_ms = 0U;
          switched = 1U;
          break;
        }
      }

      if (switched == 0U) {
        if (g_seek_visible_since_ms == 0U) {
          g_seek_visible_since_ms = now;
        }

        /* Fallback: if something is visible stably for a short time, lock it
         * so scan does not keep running forever on borderline association. */
        if ((now - g_seek_visible_since_ms) >= SEEK_ACCEPT_VISIBLE_MS) {
          for (uint8_t i = 0U; i < objs.count; i++) {
            if (objs.objects[i].valid != 0U) {
              g_selected_object_id = g_obj_ids[i];
              g_seek_other_object = 0U;
              g_seek_has_ref = 0U;
              g_selected_missing_since_ms = 0U;
              g_seek_enter_ms = 0U;
              g_seek_visible_since_ms = 0U;
              switched = 1U;
              break;
            }
          }
        }
      } else {
        g_seek_visible_since_ms = 0U;
      }

      if ((switched == 0U) && (g_seek_has_ref == 0U) && (objs.count > 1U)) {
        /* Fallback: if no reference exists, allow switching to next visible. */
        g_selected_object_id = pick_next_visible_id(&objs, g_selected_object_id);
        g_seek_other_object = 0U;
        g_seek_has_ref = 0U;
        g_selected_missing_since_ms = 0U;
        g_seek_enter_ms = 0U;
        g_seek_visible_since_ms = 0U;
      }
    } else {
      g_seek_visible_since_ms = 0U;
    }

    {
      int8_t sel_idx = find_obj_index_by_id(&objs, g_selected_object_id);
      if (sel_idx >= 0) {
        g_selected_missing_since_ms = 0U;
      } else if ((objs.count > 0U) && (g_seek_other_object == 0U)) {
        if (g_selected_missing_since_ms == 0U) {
          g_selected_missing_since_ms = now;
        }

        if ((now - g_selected_missing_since_ms) >= SELECT_FALLBACK_LAG_MS) {
          g_selected_object_id = g_obj_ids[0];
          sel_idx = 0;
          g_selected_missing_since_ms = 0U;
        }
      } else {
        g_selected_missing_since_ms = 0U;
      }
      g_selected_object = (sel_idx < 0) ? 0xFFU : (uint8_t)sel_idx;
    }

    if (g_selected_object == 0xFFU) {
      detection_from_object(&det, &objs, 0xFFU);
      det.target_found = 0U;
      det.min_x = -1;
      det.max_x = -1;
      det.min_y = -1;
      det.max_y = -1;
      det.centroid_x = -1.0f;
      det.centroid_y = -1.0f;
    } else {
      detection_from_object(&det, &objs, g_selected_object);
    }

    if ((g_seek_other_object != 0U) && (objs.count <= 1U)) {
      uint32_t seek_ms = now - g_seek_enter_ms;
      if (seek_ms < SEEK_PRE_SCAN_WAIT_MS) {
        /* Grace window: keep tracking any currently visible object while
         * waiting for another object to appear. */
        if ((det.target_found == 0U) && (objs.count > 0U) && (objs.objects[0].valid != 0U)) {
          detection_from_object(&det, &objs, 0U);
        }
      } else {
        /* After grace window, force lost-target behavior to trigger scan. */
        det.target_found = 0U;
        det.min_x = -1;
        det.max_x = -1;
        det.min_y = -1;
        det.max_y = -1;
        det.centroid_x = -1.0f;
        det.centroid_y = -1.0f;
      }
    }
#if THERMAL_SERVO_TRACK_TEST
    Tracking_UpdateFromDetection(&det);
#endif
    Thermal_UpscaleBilinear8x8(frame, upscaled, UPSCALE_W, UPSCALE_H);
#if ST7735_CN8_SPI_LIVE_VIEW
    tft_render_toggle ^= 1U;
    if (tft_render_toggle != 0U) {
      ST7735_CN8_RenderFrame32x32(upscaled, &objs, g_selected_object);
    }
#endif

#if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
    if ((int32_t)(now - next_up_tx_ms) >= 0) {
      next_up_tx_ms = now + UPSCALED_STREAM_PERIOD_MS;
      uart_send_um64_packet(up_seq++, upscaled, &objs, g_selected_object, (int16_t)(Servo_GetPan() * 10.0f));
    }
#elif !ST7735_CN8_SPI_LIVE_VIEW
    uart_send("BEGIN\r\n");
    uart_send_frame_csv(frame);
    uart_send_meta_csv(&det, Servo_GetPan());
    uart_send("END\r\n");
#endif
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
          g_next_object_request = 1U;
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
