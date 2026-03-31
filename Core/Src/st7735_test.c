#include "st7735_test.h"

/*
 * Pin map for ST7735S test (Nucleo-F446ZE Arduino headers):
 *   SCK  -> PE13 (D3)
 *   MOSI -> PE11 (D5)
 *   CS   -> PF14 (D4)
 *   DC   -> PF15 (D2)
 *   RST  -> PE9  (D6)
 */
#define TFT_SCK_GPIO_Port GPIOE
#define TFT_SCK_Pin       GPIO_PIN_13

#define TFT_MOSI_GPIO_Port GPIOE
#define TFT_MOSI_Pin       GPIO_PIN_11

#define TFT_CS_GPIO_Port GPIOF
#define TFT_CS_Pin       GPIO_PIN_14

#define TFT_DC_GPIO_Port GPIOF
#define TFT_DC_Pin       GPIO_PIN_15

#define TFT_RST_GPIO_Port GPIOE
#define TFT_RST_Pin       GPIO_PIN_9

#define ST7735_WIDTH  128U
#define ST7735_HEIGHT 128U

#define ST7735_SWRESET 0x01U
#define ST7735_SLPOUT  0x11U
#define ST7735_COLMOD  0x3AU
#define ST7735_MADCTL  0x36U
#define ST7735_CASET   0x2AU
#define ST7735_RASET   0x2BU
#define ST7735_RAMWR   0x2CU
#define ST7735_NORON   0x13U
#define ST7735_DISPON  0x29U

#define RGB565_BLACK 0x0000U
#define RGB565_RED   0xF800U
#define RGB565_WHITE 0xFFFFU

static inline void pin_hi(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static inline void pin_lo(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static inline void spi_delay(void)
{
    __NOP();
    __NOP();
    __NOP();
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

static void st7735_write_data_u8(uint8_t data)
{
    pin_lo(TFT_CS_GPIO_Port, TFT_CS_Pin);
    pin_hi(TFT_DC_GPIO_Port, TFT_DC_Pin);
    soft_spi_write_u8(data);
    pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

static void st7735_write_data_u16(uint16_t data)
{
    st7735_write_data_u8((uint8_t)((data >> 8) & 0xFFU));
    st7735_write_data_u8((uint8_t)(data & 0xFFU));
}

static void st7735_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    st7735_write_cmd(ST7735_CASET);
    st7735_write_data_u16(x0);
    st7735_write_data_u16(x1);

    st7735_write_cmd(ST7735_RASET);
    st7735_write_data_u16(y0);
    st7735_write_data_u16(y1);

    st7735_write_cmd(ST7735_RAMWR);
}

static void st7735_fill_screen(uint16_t color)
{
    st7735_set_window(0U, 0U, ST7735_WIDTH - 1U, ST7735_HEIGHT - 1U);
    for (uint32_t i = 0U; i < (ST7735_WIDTH * ST7735_HEIGHT); i++) {
        st7735_write_data_u16(color);
    }
}

static void st7735_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0U))
        return;
    if ((x + w) > ST7735_WIDTH)
        w = (uint16_t)(ST7735_WIDTH - x);

    st7735_set_window(x, y, (uint16_t)(x + w - 1U), y);
    for (uint16_t i = 0U; i < w; i++) {
        st7735_write_data_u16(color);
    }
}

static void st7735_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (h == 0U))
        return;
    if ((y + h) > ST7735_HEIGHT)
        h = (uint16_t)(ST7735_HEIGHT - y);

    st7735_set_window(x, y, x, (uint16_t)(y + h - 1U));
    for (uint16_t i = 0U; i < h; i++) {
        st7735_write_data_u16(color);
    }
}

static void st7735_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U))
        return;
    st7735_hline(x, y, w, color);
    st7735_hline(x, (uint16_t)(y + h - 1U), w, color);
    st7735_vline(x, y, h, color);
    st7735_vline((uint16_t)(x + w - 1U), y, h, color);
}

void ST7735_Test_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = TFT_SCK_Pin | TFT_MOSI_Pin | TFT_RST_Pin;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = TFT_CS_Pin | TFT_DC_Pin;
    HAL_GPIO_Init(GPIOF, &gpio);

    pin_hi(TFT_CS_GPIO_Port, TFT_CS_Pin);
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
    st7735_write_data_u8(0x05U); /* 16-bit/pixel RGB565 */

    st7735_write_cmd(ST7735_MADCTL);
    st7735_write_data_u8(0x00U); /* default orientation */

    st7735_write_cmd(ST7735_NORON);
    HAL_Delay(10);

    st7735_write_cmd(ST7735_DISPON);
    HAL_Delay(120);
}

void ST7735_Test_DrawBoxDemo(void)
{
    st7735_fill_screen(RGB565_BLACK);
    st7735_rect(16U, 16U, 96U, 96U, RGB565_RED);
    st7735_rect(24U, 24U, 80U, 80U, RGB565_WHITE);
}
