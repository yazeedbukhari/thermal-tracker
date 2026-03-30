#include "st7735_cn8_spi_test.h"

/*
 * NUCLEO-F446ZE Arduino wiring:
 *   Safer SPI path (avoids PA7 servo conflict) using SPI_B on CN7:
 *   TFT SCL/SCK -> D23 (PB3, SPI3_SCK)
 *   TFT SDA/MOSI -> D22 (PB5, SPI3_MOSI)
 *
 *   Control pins:
 *   TFT CS -> D24 (PA4, SPI_B_NSS-style GPIO chip-select)
 *   TFT DC/A0 -> D9 (PD15, GPIO)
 *   TFT RST/RES -> D8 (PF12, GPIO)
 *
 *   Power comes from CN8/CN7 3V3 and GND pins.
 */

#define TFT_SCK_GPIO_Port       GPIOB
#define TFT_SCK_Pin             GPIO_PIN_3
#define TFT_MOSI_GPIO_Port      GPIOB
#define TFT_MOSI_Pin            GPIO_PIN_5

#define TFT_CS_GPIO_Port        GPIOA
#define TFT_CS_Pin              GPIO_PIN_4
#define TFT_DC_GPIO_Port        GPIOD
#define TFT_DC_Pin              GPIO_PIN_15
#define TFT_RST_GPIO_Port       GPIOF
#define TFT_RST_Pin             GPIO_PIN_12

#define ST7735_WIDTH            128U
#define ST7735_HEIGHT           128U
/* Tune these per panel variant if one edge shows garbage pixels. */
#define ST7735_X_OFFSET         2U
#define ST7735_Y_OFFSET         1U
/* 0 = normal mapping, 1 = swap X/Y addressing for rotated panels. */
#define ST7735_SWAP_XY          0U
/* BGR bit is commonly needed on ST7735S modules. */
#define ST7735_MADCTL_VALUE     0x08U

#define ST7735_SWRESET          0x01U
#define ST7735_SLPOUT           0x11U
#define ST7735_COLMOD           0x3AU
#define ST7735_MADCTL           0x36U
#define ST7735_CASET            0x2AU
#define ST7735_RASET            0x2BU
#define ST7735_RAMWR            0x2CU
#define ST7735_NORON            0x13U
#define ST7735_DISPON           0x29U

#define RGB565_BLACK            0x0000U
#define RGB565_RED              0xF800U
#define RGB565_WHITE            0xFFFFU
#define RGB565_YELLOW           0xFFE0U
#define RGB565_LIME             0x07E0U
#define RGB565_CYAN             0x07FFU

#define THERMAL_Q_MIN_C         18.0f
#define THERMAL_Q_MAX_C         35.0f
#define TFT_VIEW_W              64U
#define TFT_VIEW_H              64U
#define TFT_VIEW_X              ((ST7735_WIDTH - TFT_VIEW_W) / 2U)
#define TFT_VIEW_Y              ((ST7735_HEIGHT - TFT_VIEW_H) / 2U)
/* Set >0 only if your panel needs slower bit-bang edges. */
#define SPI_DELAY_NOPS          0U

static inline void pin_hi(GPIO_TypeDef *port, uint16_t pin)
{
  port->BSRR = pin;
}

static inline void pin_lo(GPIO_TypeDef *port, uint16_t pin)
{
  port->BSRR = ((uint32_t)pin << 16U);
}

static inline void spi_delay(void)
{
#if SPI_DELAY_NOPS > 0U
  for (uint8_t i = 0U; i < SPI_DELAY_NOPS; i++) {
    __NOP();
  }
#endif
}

static void soft_spi_write_u8(uint8_t v)
{
  for (uint8_t i = 0U; i < 8U; i++) {
    if ((v & 0x80U) != 0U) {
      pin_hi(TFT_MOSI_GPIO_Port, TFT_MOSI_Pin);
    } else {
      pin_lo(TFT_MOSI_GPIO_Port, TFT_MOSI_Pin);
    }

    pin_hi(TFT_SCK_GPIO_Port, TFT_SCK_Pin);
    spi_delay();
    pin_lo(TFT_SCK_GPIO_Port, TFT_SCK_Pin);
    spi_delay();
    v <<= 1;
  }
}

static void st7735_write_cmd(uint8_t cmd)
{
  pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
  pin_lo(TFT_DC_GPIO_Port, TFT_DC_Pin);
  soft_spi_write_u8(cmd);
  pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

static void st7735_write_data(const uint8_t *data, uint16_t len)
{
  pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
  pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
  for (uint16_t i = 0U; i < len; i++) {
    soft_spi_write_u8(data[i]);
  }
  pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

static void st7735_data_begin(void)
{
  pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
  pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
}

static void st7735_data_end(void)
{
  pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

static inline void st7735_stream_u16(uint16_t value)
{
  soft_spi_write_u8((uint8_t)((value >> 8) & 0xFFU));
  soft_spi_write_u8((uint8_t)(value & 0xFFU));
}

static void st7735_write_data_u16(uint16_t value)
{
  uint8_t bytes[2];
  bytes[0] = (uint8_t)((value >> 8) & 0xFFU);
  bytes[1] = (uint8_t)(value & 0xFFU);
  st7735_write_data(bytes, 2U);
}

static void st7735_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint16_t xa = x0;
  uint16_t xb = x1;
  uint16_t ya = y0;
  uint16_t yb = y1;

#if ST7735_SWAP_XY
  xa = y0;
  xb = y1;
  ya = x0;
  yb = x1;
#endif

  xa = (uint16_t)(xa + ST7735_X_OFFSET);
  xb = (uint16_t)(xb + ST7735_X_OFFSET);
  ya = (uint16_t)(ya + ST7735_Y_OFFSET);
  yb = (uint16_t)(yb + ST7735_Y_OFFSET);

  st7735_write_cmd(ST7735_CASET);
  st7735_write_data_u16(xa);
  st7735_write_data_u16(xb);

  st7735_write_cmd(ST7735_RASET);
  st7735_write_data_u16(ya);
  st7735_write_data_u16(yb);

  st7735_write_cmd(ST7735_RAMWR);
}

static void st7735_fill_screen(uint16_t color)
{
  st7735_set_window(0U, 0U, ST7735_WIDTH - 1U, ST7735_HEIGHT - 1U);
  st7735_data_begin();
  for (uint32_t i = 0U; i < (ST7735_WIDTH * ST7735_HEIGHT); i++) {
    st7735_stream_u16(color);
  }
  st7735_data_end();
}

static void st7735_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0U)) return;
  if ((x + w) > ST7735_WIDTH) w = (uint16_t)(ST7735_WIDTH - x);

  st7735_set_window(x, y, (uint16_t)(x + w - 1U), y);
  st7735_data_begin();
  for (uint16_t i = 0U; i < w; i++) {
    st7735_stream_u16(color);
  }
  st7735_data_end();
}

static void st7735_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (h == 0U)) return;
  if ((y + h) > ST7735_HEIGHT) h = (uint16_t)(ST7735_HEIGHT - y);

  st7735_set_window(x, y, x, (uint16_t)(y + h - 1U));
  st7735_data_begin();
  for (uint16_t i = 0U; i < h; i++) {
    st7735_stream_u16(color);
  }
  st7735_data_end();
}

static void st7735_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((w == 0U) || (h == 0U)) return;
  st7735_hline(x, y, w, color);
  st7735_hline(x, (uint16_t)(y + h - 1U), w, color);
  st7735_vline(x, y, h, color);
  st7735_vline((uint16_t)(x + w - 1U), y, h, color);
}


static uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint16_t)(((uint16_t)(r & 0xF8U) << 8) |
                    ((uint16_t)(g & 0xFCU) << 3) |
                    ((uint16_t)(b & 0xF8U) >> 3));
}

static uint16_t heat_color_from_u8(uint8_t v)
{
  uint8_t r, g, b;

  if (v < 64U) {
    /* Dark blue -> purple */
    uint8_t t = (uint8_t)(v * 4U);
    r = t / 2U;
    g = 0U;
    b = (uint8_t)(80U + (t / 2U));
  } else if (v < 128U) {
    /* Purple -> red */
    uint8_t t = (uint8_t)((v - 64U) * 4U);
    r = (uint8_t)(128U + (t / 2U));
    g = 0U;
    b = (uint8_t)(160U - t / 2U);
  } else if (v < 192U) {
    /* Red -> orange/yellow */
    uint8_t t = (uint8_t)((v - 128U) * 4U);
    r = 255U;
    g = (uint8_t)(t / 2U);
    b = 0U;
  } else {
    /* Yellow -> near white */
    uint8_t t = (uint8_t)((v - 192U) * 4U);
    r = 255U;
    g = 255U;
    b = (uint8_t)(t / 2U);
  }

  return rgb888_to_565(r, g, b);
}

static uint8_t quantize_temp_u8(float t)
{
  float n = (t - THERMAL_Q_MIN_C) / (THERMAL_Q_MAX_C - THERMAL_Q_MIN_C);
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  return (uint8_t)(n * 255.0f);
}

static void st7735_draw_cross(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (y < 0) || (x >= (int16_t)ST7735_WIDTH) || (y >= (int16_t)ST7735_HEIGHT)) {
    return;
  }

  int16_t x0 = (x > 1) ? (x - 1) : 0;
  int16_t y0 = (y > 1) ? (y - 1) : 0;
  int16_t x1 = (x < ((int16_t)ST7735_WIDTH - 2)) ? (x + 1) : ((int16_t)ST7735_WIDTH - 1);
  int16_t y1 = (y < ((int16_t)ST7735_HEIGHT - 2)) ? (y + 1) : ((int16_t)ST7735_HEIGHT - 1);

  st7735_hline((uint16_t)x0, (uint16_t)y, (uint16_t)(x1 - x0 + 1), color);
  st7735_vline((uint16_t)x, (uint16_t)y0, (uint16_t)(y1 - y0 + 1), color);
}


void ST7735_CN8_Test_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  gpio.Pin = TFT_SCK_Pin | TFT_MOSI_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = TFT_CS_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = TFT_DC_Pin;
  HAL_GPIO_Init(GPIOD, &gpio);

  gpio.Pin = TFT_RST_Pin;
  HAL_GPIO_Init(GPIOF, &gpio);

  pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
  pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
  pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);

  pin_lo(TFT_SCK_GPIO_Port, TFT_SCK_Pin);
  pin_lo(TFT_MOSI_GPIO_Port, TFT_MOSI_Pin);

  pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(5);
  pin_lo(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(20);
  pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(120);

  st7735_write_cmd(ST7735_SWRESET);
  HAL_Delay(150);

  st7735_write_cmd(ST7735_SLPOUT);
  HAL_Delay(120);

  st7735_write_cmd(ST7735_COLMOD);
  {
    const uint8_t colmod = 0x05U; /* RGB565 */
    st7735_write_data(&colmod, 1U);
  }

  st7735_write_cmd(ST7735_MADCTL);
  {
    const uint8_t madctl = ST7735_MADCTL_VALUE;
    st7735_write_data(&madctl, 1U);
  }

  st7735_write_cmd(ST7735_NORON);
  HAL_Delay(10);
  st7735_write_cmd(ST7735_DISPON);
  HAL_Delay(120);

  st7735_fill_screen(RGB565_BLACK);
}

void ST7735_CN8_Test_DrawBox(void)
{
  st7735_fill_screen(RGB565_BLACK);
  st7735_rect(12U, 12U, 104U, 104U, RGB565_RED);
  st7735_rect(20U, 20U, 88U, 88U, RGB565_WHITE);
}

void ST7735_CN8_RenderFrame32x32(
  const float *upscaled_map,
  const ThermalObjectsResult *objs,
  uint8_t selected_idx
)
{
  const int16_t scale_x = (int16_t)(TFT_VIEW_W / 8U);
  const int16_t scale_y = (int16_t)(TFT_VIEW_H / 8U);

  /* Render a centered 64x64 viewport to balance detail and loop latency. */
  st7735_set_window(TFT_VIEW_X, TFT_VIEW_Y, (uint16_t)(TFT_VIEW_X + TFT_VIEW_W - 1U), (uint16_t)(TFT_VIEW_Y + TFT_VIEW_H - 1U));
  st7735_data_begin();
  for (uint16_t y = 0U; y < TFT_VIEW_H; y++) {
    for (uint16_t x = 0U; x < TFT_VIEW_W; x++) {
      float t = upscaled_map[(y * TFT_VIEW_W) + x];
      uint8_t q = quantize_temp_u8(t);
      st7735_stream_u16(heat_color_from_u8(q));
    }
  }
  st7735_data_end();

  if (objs == NULL) {
    return;
  }

  uint8_t has_selected = 0U;
  if ((objs->count > 0U) && (selected_idx < objs->count)) {
    has_selected = 1U;
  }

  for (uint8_t i = 0U; i < THERMAL_MAX_OBJECTS; i++) {
    const ThermalObject *obj = &objs->objects[i];
    if ((i >= objs->count) || (obj->valid == 0U) || (obj->min_x < 0) || (obj->min_y < 0)) {
      continue;
    }

    /* Map 8x8 detection space into display viewport (64x64 => factor 8). */
    int16_t bx = (int16_t)(TFT_VIEW_X + (obj->min_x * scale_x));
    int16_t by = (int16_t)(TFT_VIEW_Y + (obj->min_y * scale_y));
    int16_t bw = (int16_t)((obj->max_x - obj->min_x + 1) * scale_x);
    int16_t bh = (int16_t)((obj->max_y - obj->min_y + 1) * scale_y);

    if ((bx < 0) || (by < 0) || (bx >= (int16_t)ST7735_WIDTH) || (by >= (int16_t)ST7735_HEIGHT)) {
      continue;
    }
    if ((bw <= 0) || (bh <= 0)) {
      continue;
    }
    if ((bx + bw) > (int16_t)ST7735_WIDTH) bw = (int16_t)ST7735_WIDTH - bx;
    if ((by + bh) > (int16_t)ST7735_HEIGHT) bh = (int16_t)ST7735_HEIGHT - by;

    if ((has_selected != 0U) && (i == selected_idx)) {
      st7735_rect((uint16_t)bx, (uint16_t)by, (uint16_t)bw, (uint16_t)bh, RGB565_LIME);
      st7735_draw_cross((int16_t)(TFT_VIEW_X + (obj->centroid_x * (float)scale_x)), (int16_t)(TFT_VIEW_Y + (obj->centroid_y * (float)scale_y)), RGB565_CYAN);
    } else {
      st7735_rect((uint16_t)bx, (uint16_t)by, (uint16_t)bw, (uint16_t)bh, RGB565_YELLOW);
      st7735_draw_cross((int16_t)(TFT_VIEW_X + (obj->centroid_x * (float)scale_x)), (int16_t)(TFT_VIEW_Y + (obj->centroid_y * (float)scale_y)), RGB565_WHITE);
    }
  }

}
