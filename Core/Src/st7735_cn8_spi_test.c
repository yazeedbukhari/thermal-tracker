#include "st7735_cn8_spi_test.h"
#include "config.h"

/*
 * NUCLEO-F446ZE Arduino wiring:
 *   TFT SCL/SCK  -> D23 (PB3, SPI3_SCK)
 *   TFT SDA/MOSI -> D22 (PB5, SPI3_MOSI)
 *
 *   Control pins:
 *   TFT CS  -> D24 (PA4, GPIO)
 *   TFT DC  -> D9  (PD15, GPIO)
 *   TFT RST -> D8  (PF12, GPIO)
 */

#define TFT_SCK_GPIO_Port  GPIOB
#define TFT_SCK_Pin        GPIO_PIN_3
#define TFT_MOSI_GPIO_Port GPIOB
#define TFT_MOSI_Pin       GPIO_PIN_5

#define TFT_CS_GPIO_Port   GPIOA
#define TFT_CS_Pin         GPIO_PIN_4
#define TFT_DC_GPIO_Port   GPIOD
#define TFT_DC_Pin         GPIO_PIN_15
#define TFT_RST_GPIO_Port  GPIOF
#define TFT_RST_Pin        GPIO_PIN_12

#define ST7735_WIDTH       128U
#define ST7735_HEIGHT      128U
#define ST7735_X_OFFSET    2U
#define ST7735_Y_OFFSET    1U
#define ST7735_SWAP_XY     0U
#define ST7735_MADCTL_VALUE 0x48U

#define ST7735_SWRESET     0x01U
#define ST7735_SLPOUT      0x11U
#define ST7735_COLMOD      0x3AU
#define ST7735_MADCTL      0x36U
#define ST7735_CASET       0x2AU
#define ST7735_RASET       0x2BU
#define ST7735_RAMWR       0x2CU
#define ST7735_NORON       0x13U
#define ST7735_DISPON      0x29U

#define RGB565_BLACK       0x0000U
#define RGB565_RED         0xF800U
#define RGB565_WHITE       0xFFFFU
#define RGB565_YELLOW      0xFFE0U
#define RGB565_LIME        0x07E0U
#define RGB565_CYAN        0x07FFU

#define THERMAL_Q_MIN_C    18.0f
#define THERMAL_Q_MAX_C    35.0f
#define THERMAL_Q_MIN_SPAN 4.0f
#define THERMAL_Q_ALPHA    0.25f
#define THERMAL_SHARP_K    0.35f
#define TFT_SRC_W          32U
#define TFT_SRC_H          32U
#define TFT_VIEW_W         128U
#define TFT_VIEW_H         128U
#define TFT_VIEW_X         ((ST7735_WIDTH - TFT_VIEW_W) / 2U)
#define TFT_VIEW_Y         ((ST7735_HEIGHT - TFT_VIEW_H) / 2U)

#define TFT_SPI_TIMEOUT_MS 20U
#define TFT_DMA_PIXELS     (TFT_VIEW_W * TFT_VIEW_H)
#define TFT_DMA_BYTES      (TFT_DMA_PIXELS * 2U)

static uint16_t s_framebuf[2][TFT_DMA_PIXELS];
static float s_dyn_lo = THERMAL_Q_MIN_C;
static float s_dyn_hi = THERMAL_Q_MAX_C;
static volatile uint8_t s_dma_busy      = 0U;
static volatile uint8_t s_tx_active_idx = 0U;
static volatile uint8_t s_pending_valid = 0U;
static volatile uint8_t s_pending_idx   = 0U;
static volatile uint32_t s_frames_queued  = 0U;
static volatile uint32_t s_frames_dropped = 0U;

static inline void pin_hi(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = pin;
}

static inline void pin_lo(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = ((uint32_t)pin << 16U);
}

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static HAL_StatusTypeDef spi_tx_blocking(const uint8_t *data, uint16_t len)
{
    return HAL_SPI_Transmit(&hspi3_tft, (uint8_t *)data, len, TFT_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef st7735_write_cmd(uint8_t cmd)
{
    HAL_StatusTypeDef st;
    pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
    pin_lo(TFT_DC_GPIO_Port, TFT_DC_Pin);
    st = spi_tx_blocking(&cmd, 1U);
    pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
    return st;
}

static HAL_StatusTypeDef st7735_write_data(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st;
    pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
    pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
    st = spi_tx_blocking(data, len);
    pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
    return st;
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

static HAL_StatusTypeDef st7735_write_data_u16(uint16_t value)
{
    uint8_t bytes[2];
    bytes[0] = (uint8_t)((value >> 8) & 0xFFU);
    bytes[1] = (uint8_t)(value & 0xFFU);
    return st7735_write_data(bytes, 2U);
}

static HAL_StatusTypeDef st7735_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
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

    if (st7735_write_cmd(ST7735_CASET) != HAL_OK) {
        return HAL_ERROR;
    }
    if (st7735_write_data_u16(xa) != HAL_OK) {
        return HAL_ERROR;
    }
    if (st7735_write_data_u16(xb) != HAL_OK) {
        return HAL_ERROR;
    }

    if (st7735_write_cmd(ST7735_RASET) != HAL_OK) {
        return HAL_ERROR;
    }
    if (st7735_write_data_u16(ya) != HAL_OK) {
        return HAL_ERROR;
    }
    if (st7735_write_data_u16(yb) != HAL_OK) {
        return HAL_ERROR;
    }

    if (st7735_write_cmd(ST7735_RAMWR) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void st7735_stream_color_blocking(uint32_t count, uint16_t color)
{
    uint8_t chunk[64];
    uint8_t hi = (uint8_t)((color >> 8) & 0xFFU);
    uint8_t lo = (uint8_t)(color & 0xFFU);

    for (uint16_t i = 0U; i < 32U; i++) {
        chunk[2U * i]       = hi;
        chunk[(2U * i) + 1] = lo;
    }

    while (count > 0U) {
        uint16_t px_now = (count > 32U) ? 32U : (uint16_t)count;
        if (spi_tx_blocking(chunk, (uint16_t)(px_now * 2U)) != HAL_OK) {
            return;
        }
        count -= px_now;
    }
}

static void st7735_fill_screen(uint16_t color)
{
    if (st7735_set_window(0U, 0U, ST7735_WIDTH - 1U, ST7735_HEIGHT - 1U) != HAL_OK) {
        return;
    }
    st7735_data_begin();
    st7735_stream_color_blocking((uint32_t)(ST7735_WIDTH * ST7735_HEIGHT), color);
    st7735_data_end();
}

static void st7735_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0U)) {
        return;
    }
    if ((x + w) > ST7735_WIDTH) {
        w = (uint16_t)(ST7735_WIDTH - x);
    }

    if (st7735_set_window(x, y, (uint16_t)(x + w - 1U), y) != HAL_OK) {
        return;
    }

    st7735_data_begin();
    st7735_stream_color_blocking(w, color);
    st7735_data_end();
}

static void st7735_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (h == 0U)) {
        return;
    }
    if ((y + h) > ST7735_HEIGHT) {
        h = (uint16_t)(ST7735_HEIGHT - y);
    }

    if (st7735_set_window(x, y, x, (uint16_t)(y + h - 1U)) != HAL_OK) {
        return;
    }

    st7735_data_begin();
    st7735_stream_color_blocking(h, color);
    st7735_data_end();
}

static void st7735_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U)) {
        return;
    }
    st7735_hline(x, y, w, color);
    st7735_hline(x, (uint16_t)(y + h - 1U), w, color);
    st7735_vline(x, y, h, color);
    st7735_vline((uint16_t)(x + w - 1U), y, h, color);
}

static uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((uint16_t)(r & 0xF8U) << 8) | ((uint16_t)(g & 0xFCU) << 3) |
                      ((uint16_t)(b & 0xF8U) >> 3));
}

static uint16_t heat_color_from_u8(uint8_t v)
{
    uint8_t r, g, b;

    if (v < 64U) {
        uint8_t t = (uint8_t)(v * 4U);
        r         = t / 2U;
        g         = 0U;
        b         = (uint8_t)(80U + (t / 2U));
    } else if (v < 128U) {
        uint8_t t = (uint8_t)((v - 64U) * 4U);
        r         = (uint8_t)(128U + (t / 2U));
        g         = 0U;
        b         = (uint8_t)(160U - t / 2U);
    } else if (v < 192U) {
        uint8_t t = (uint8_t)((v - 128U) * 4U);
        r         = 255U;
        g         = (uint8_t)(t / 2U);
        b         = 0U;
    } else {
        uint8_t t = (uint8_t)((v - 192U) * 4U);
        r         = 255U;
        g         = 255U;
        b         = (uint8_t)(t / 2U);
    }

    return rgb888_to_565(r, g, b);
}

static uint8_t quantize_temp_u8(float t)
{
    float n = (t - THERMAL_Q_MIN_C) / (THERMAL_Q_MAX_C - THERMAL_Q_MIN_C);
    if (n < 0.0f) {
        n = 0.0f;
    }
    if (n > 1.0f) {
        n = 1.0f;
    }
    return (uint8_t)(n * 255.0f);
}

static uint8_t quantize_temp_dynamic_u8(float t, float lo, float hi)
{
    float n;
    if (hi <= lo) {
        return quantize_temp_u8(t);
    }
    n = (t - lo) / (hi - lo);
    if (n < 0.0f) {
        n = 0.0f;
    }
    if (n > 1.0f) {
        n = 1.0f;
    }
    return (uint8_t)(n * 255.0f);
}

static inline uint16_t rgb565_to_be(uint16_t c)
{
    return (uint16_t)((c << 8U) | (c >> 8U));
}

static void fb_hline(uint16_t *fb, int16_t x, int16_t y, int16_t w, uint16_t color)
{
    if ((w <= 0) || (y < 0) || (y >= (int16_t)TFT_VIEW_H)) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if ((x + w) > (int16_t)TFT_VIEW_W) {
        w = (int16_t)TFT_VIEW_W - x;
    }
    if (w <= 0) {
        return;
    }

    uint16_t color_be = rgb565_to_be(color);
    uint32_t base     = (uint32_t)y * TFT_VIEW_W;
    for (int16_t i = 0; i < w; i++) {
        fb[base + (uint32_t)(x + i)] = color_be;
    }
}

static void fb_vline(uint16_t *fb, int16_t x, int16_t y, int16_t h, uint16_t color)
{
    if ((h <= 0) || (x < 0) || (x >= (int16_t)TFT_VIEW_W)) {
        return;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if ((y + h) > (int16_t)TFT_VIEW_H) {
        h = (int16_t)TFT_VIEW_H - y;
    }
    if (h <= 0) {
        return;
    }

    uint16_t color_be = rgb565_to_be(color);
    for (int16_t i = 0; i < h; i++) {
        fb[(uint32_t)(y + i) * TFT_VIEW_W + (uint32_t)x] = color_be;
    }
}

static void fb_rect(uint16_t *fb, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if ((w <= 0) || (h <= 0)) {
        return;
    }
    fb_hline(fb, x, y, w, color);
    fb_hline(fb, x, (int16_t)(y + h - 1), w, color);
    fb_vline(fb, x, y, h, color);
    fb_vline(fb, (int16_t)(x + w - 1), y, h, color);
}

static void fb_cross(uint16_t *fb, int16_t x, int16_t y, uint16_t color)
{
    int16_t x0 = (x > 1) ? (x - 1) : 0;
    int16_t y0 = (y > 1) ? (y - 1) : 0;
    int16_t x1 = (x < ((int16_t)TFT_VIEW_W - 2)) ? (x + 1) : ((int16_t)TFT_VIEW_W - 1);
    int16_t y1 = (y < ((int16_t)TFT_VIEW_H - 2)) ? (y + 1) : ((int16_t)TFT_VIEW_H - 1);

    fb_hline(fb, x0, y, (int16_t)(x1 - x0 + 1), color);
    fb_vline(fb, x, y0, (int16_t)(y1 - y0 + 1), color);
}

static void st7735_build_frame(uint8_t frame_idx, const float *upscaled_map,
                               const ThermalObjectsResult *objs, uint8_t selected_idx)
{
    uint16_t *fb = s_framebuf[frame_idx];
    const int16_t scale_x = (int16_t)(TFT_VIEW_W / 8U);
    const int16_t scale_y = (int16_t)(TFT_VIEW_H / 8U);
    float frame_min = 1000.0f;
    float frame_max = -1000.0f;
    float target_lo;
    float target_hi;
    float span;

    for (uint16_t i = 0U; i < (uint16_t)(TFT_SRC_W * TFT_SRC_H); i++) {
        float t = upscaled_map[i];
        if (t < frame_min) {
            frame_min = t;
        }
        if (t > frame_max) {
            frame_max = t;
        }
    }

    target_lo = frame_min;
    target_hi = frame_max;
    span = target_hi - target_lo;
    if (span < THERMAL_Q_MIN_SPAN) {
        float mid = 0.5f * (target_lo + target_hi);
        target_lo = mid - (0.5f * THERMAL_Q_MIN_SPAN);
        target_hi = mid + (0.5f * THERMAL_Q_MIN_SPAN);
    }
    if (target_lo < THERMAL_Q_MIN_C) {
        target_lo = THERMAL_Q_MIN_C;
    }
    if (target_hi > THERMAL_Q_MAX_C) {
        target_hi = THERMAL_Q_MAX_C;
    }
    if ((target_hi - target_lo) < THERMAL_Q_MIN_SPAN) {
        target_hi = target_lo + THERMAL_Q_MIN_SPAN;
        if (target_hi > THERMAL_Q_MAX_C) {
            target_hi = THERMAL_Q_MAX_C;
            target_lo = target_hi - THERMAL_Q_MIN_SPAN;
            if (target_lo < THERMAL_Q_MIN_C) {
                target_lo = THERMAL_Q_MIN_C;
            }
        }
    }

    s_dyn_lo += THERMAL_Q_ALPHA * (target_lo - s_dyn_lo);
    s_dyn_hi += THERMAL_Q_ALPHA * (target_hi - s_dyn_hi);

    for (uint16_t y = 0U; y < TFT_VIEW_H; y++) {
        float gy = ((float)y) * (float)(TFT_SRC_H - 1U) / (float)(TFT_VIEW_H - 1U);
        uint16_t sy0 = (uint16_t)gy;
        uint16_t sy1 = (sy0 < (TFT_SRC_H - 1U)) ? (uint16_t)(sy0 + 1U) : sy0;
        float dy = gy - (float)sy0;

        for (uint16_t x = 0U; x < TFT_VIEW_W; x++) {
            float gx = ((float)x) * (float)(TFT_SRC_W - 1U) / (float)(TFT_VIEW_W - 1U);
            uint16_t sx0 = (uint16_t)gx;
            uint16_t sx1 = (sx0 < (TFT_SRC_W - 1U)) ? (uint16_t)(sx0 + 1U) : sx0;
            float dx = gx - (float)sx0;

            uint32_t idx00 = ((uint32_t)sy0 * TFT_SRC_W) + sx0;
            uint32_t idx10 = ((uint32_t)sy0 * TFT_SRC_W) + sx1;
            uint32_t idx01 = ((uint32_t)sy1 * TFT_SRC_W) + sx0;
            uint32_t idx11 = ((uint32_t)sy1 * TFT_SRC_W) + sx1;
            uint32_t didx = ((uint32_t)y * TFT_VIEW_W) + x;

            float t00 = upscaled_map[idx00];
            float t10 = upscaled_map[idx10];
            float t01 = upscaled_map[idx01];
            float t11 = upscaled_map[idx11];
            float t0 = t00 + (dx * (t10 - t00));
            float t1 = t01 + (dx * (t11 - t01));
            float t = t0 + (dy * (t1 - t0));

            if ((sx0 > 0U) && (sx0 < (TFT_SRC_W - 1U)) &&
                (sy0 > 0U) && (sy0 < (TFT_SRC_H - 1U))) {
                float n4 = upscaled_map[idx00 - 1U] +
                           upscaled_map[idx00 + 1U] +
                           upscaled_map[idx00 - TFT_SRC_W] +
                           upscaled_map[idx00 + TFT_SRC_W];
                float avg4 = 0.25f * n4;
                t = t + (THERMAL_SHARP_K * (t - avg4));
            }

            {
                uint8_t q = quantize_temp_dynamic_u8(t, s_dyn_lo, s_dyn_hi);
                fb[didx] = rgb565_to_be(heat_color_from_u8(q));
            }
        }
    }

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

        int16_t bx = (int16_t)(obj->min_x * scale_x);
        int16_t by = (int16_t)(obj->min_y * scale_y);
        int16_t bw = (int16_t)((obj->max_x - obj->min_x + 1) * scale_x);
        int16_t bh = (int16_t)((obj->max_y - obj->min_y + 1) * scale_y);
        int16_t cx = (int16_t)(obj->centroid_x * (float)scale_x);
        int16_t cy = (int16_t)(obj->centroid_y * (float)scale_y);

        if ((bx < 0) || (by < 0) || (bx >= (int16_t)TFT_VIEW_W) || (by >= (int16_t)TFT_VIEW_H)) {
            continue;
        }
        if ((bw <= 0) || (bh <= 0)) {
            continue;
        }
        if ((bx + bw) > (int16_t)TFT_VIEW_W) {
            bw = (int16_t)TFT_VIEW_W - bx;
        }
        if ((by + bh) > (int16_t)TFT_VIEW_H) {
            bh = (int16_t)TFT_VIEW_H - by;
        }

        if ((has_selected != 0U) && (i == selected_idx)) {
            fb_rect(fb, bx, by, bw, bh, RGB565_LIME);
            fb_cross(fb, cx, cy, RGB565_CYAN);
        } else {
            fb_rect(fb, bx, by, bw, bh, RGB565_YELLOW);
            fb_cross(fb, cx, cy, RGB565_WHITE);
        }
    }
}

static HAL_StatusTypeDef st7735_start_viewport_dma(uint8_t frame_idx)
{
    HAL_StatusTypeDef st;

    st = st7735_set_window(TFT_VIEW_X, TFT_VIEW_Y, (uint16_t)(TFT_VIEW_X + TFT_VIEW_W - 1U),
                           (uint16_t)(TFT_VIEW_Y + TFT_VIEW_H - 1U));
    if (st != HAL_OK) {
        return st;
    }

    st7735_data_begin();
    st = HAL_SPI_Transmit_DMA(&hspi3_tft, (uint8_t *)s_framebuf[frame_idx], (uint16_t)TFT_DMA_BYTES);
    if (st != HAL_OK) {
        st7735_data_end();
    }
    return st;
}

static void st7735_submit_frame(uint8_t frame_idx)
{
    uint8_t start_now = 0U;
    uint32_t primask  = irq_save();

    if (s_dma_busy == 0U) {
        s_dma_busy      = 1U;
        s_tx_active_idx = frame_idx;
        start_now       = 1U;
    } else {
        if (s_pending_valid != 0U) {
            s_frames_dropped++;
        }
        s_pending_idx   = frame_idx;
        s_pending_valid = 1U;
    }

    irq_restore(primask);

    if (start_now != 0U) {
        if (st7735_start_viewport_dma(frame_idx) != HAL_OK) {
            primask = irq_save();
            s_dma_busy = 0U;
            irq_restore(primask);
        } else {
            primask = irq_save();
            s_frames_queued++;
            irq_restore(primask);
        }
    }
}

void ST7735_CN8_Test_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t primask;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = 0U;

    gpio.Pin = TFT_CS_Pin;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = TFT_DC_Pin;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = TFT_RST_Pin;
    HAL_GPIO_Init(GPIOF, &gpio);

    pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
    pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
    pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);

    pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);
    HAL_Delay(5);
    pin_lo(TFT_RST_GPIO_Port, TFT_RST_Pin);
    HAL_Delay(20);
    pin_hi(TFT_RST_GPIO_Port, TFT_RST_Pin);
    HAL_Delay(120);

    (void)st7735_write_cmd(ST7735_SWRESET);
    HAL_Delay(150);

    (void)st7735_write_cmd(ST7735_SLPOUT);
    HAL_Delay(120);

    (void)st7735_write_cmd(ST7735_COLMOD);
    {
        const uint8_t colmod = 0x05U;
        (void)st7735_write_data(&colmod, 1U);
    }

    (void)st7735_write_cmd(ST7735_MADCTL);
    {
        const uint8_t madctl = ST7735_MADCTL_VALUE;
        (void)st7735_write_data(&madctl, 1U);
    }

    (void)st7735_write_cmd(ST7735_NORON);
    HAL_Delay(10);
    (void)st7735_write_cmd(ST7735_DISPON);
    HAL_Delay(120);

    primask = irq_save();
    s_dma_busy      = 0U;
    s_tx_active_idx = 0U;
    s_pending_valid = 0U;
    s_pending_idx   = 0U;
    s_frames_queued = 0U;
    s_frames_dropped = 0U;
    irq_restore(primask);

    st7735_fill_screen(RGB565_BLACK);
}

void ST7735_CN8_Test_DrawBox(void)
{
    st7735_fill_screen(RGB565_BLACK);
    st7735_rect(12U, 12U, 104U, 104U, RGB565_RED);
    st7735_rect(20U, 20U, 88U, 88U, RGB565_WHITE);
}

void ST7735_CN8_RenderFrame32x32(const float *upscaled_map, const ThermalObjectsResult *objs,
                                 uint8_t selected_idx)
{
    uint8_t build_idx;
    uint32_t primask;

    if (upscaled_map == NULL) {
        return;
    }

    primask = irq_save();
    if (s_dma_busy != 0U) {
        if (s_pending_valid != 0U) {
            build_idx = s_pending_idx;
            s_pending_valid = 0U;
            s_frames_dropped++;
        } else {
            build_idx = (uint8_t)(s_tx_active_idx ^ 1U);
        }
    } else {
        build_idx = (uint8_t)(s_tx_active_idx ^ 1U);
    }
    irq_restore(primask);

    st7735_build_frame(build_idx, upscaled_map, objs, selected_idx);
    st7735_submit_frame(build_idx);
}

void ST7735_CN8_GetRenderStats(uint32_t *queued, uint32_t *dropped)
{
    uint32_t primask = irq_save();
    if (queued != NULL) {
        *queued = s_frames_queued;
    }
    if (dropped != NULL) {
        *dropped = s_frames_dropped;
    }
    irq_restore(primask);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t start_next = 0U;
    uint8_t next_idx   = 0U;
    uint32_t primask;

    if ((hspi == NULL) || (hspi->Instance != SPI3)) {
        return;
    }

    st7735_data_end();

    primask = irq_save();
    s_dma_busy = 0U;
    if (s_pending_valid != 0U) {
        next_idx = s_pending_idx;
        s_pending_valid = 0U;
        s_tx_active_idx = next_idx;
        s_dma_busy = 1U;
        start_next = 1U;
    }
    irq_restore(primask);

    if (start_next != 0U) {
        if (st7735_start_viewport_dma(next_idx) != HAL_OK) {
            primask = irq_save();
            s_dma_busy = 0U;
            irq_restore(primask);
            st7735_data_end();
        } else {
            primask = irq_save();
            s_frames_queued++;
            irq_restore(primask);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t primask;

    if ((hspi == NULL) || (hspi->Instance != SPI3)) {
        return;
    }

    st7735_data_end();
    primask = irq_save();
    s_dma_busy = 0U;
    s_pending_valid = 0U;
    irq_restore(primask);
}
