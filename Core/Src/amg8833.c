/*
 * amg8833.c — AMG8833 thermal array sensor driver (STM32 HAL I2C)
 *
 * Implements sensor probe, initialization, raw frame read, and raw-to-Celsius
 * conversion for all 64 thermal pixels.
 *
 * Owner:
 */

#include "amg8833.h"

/* Conservative timeouts are fine at 10 FPS frame rate. */
#define AMG8833_I2C_TIMEOUT_MS     100U
#define AMG8833_I2C_READ_TIMEOUT_MS 200U

static HAL_StatusTypeDef amg8833_write8(
    I2C_HandleTypeDef *hi2c,
    uint8_t addr_7bit,
    uint8_t reg,
    uint8_t value
) {
    return HAL_I2C_Mem_Write(
        hi2c,
        AMG8833_HAL_ADDR(addr_7bit),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        AMG8833_I2C_TIMEOUT_MS
    );
}

static float amg8833_raw_pixel_to_c(uint16_t raw)
{
    int16_t value = (int16_t)(raw & 0x07FFU);
    if ((raw & 0x0800U) != 0U) {
        value = (int16_t)(-value);
    }
    return ((float)value) * 0.25f;
}

HAL_StatusTypeDef AMG8833_Probe(I2C_HandleTypeDef *hi2c, uint8_t *detected_addr_7bit)
{
    if ((hi2c == NULL) || (detected_addr_7bit == NULL)) {
        return HAL_ERROR;
    }

    const uint8_t candidates[2] = {AMG8833_ADDR_7BIT_LOW, AMG8833_ADDR_7BIT_HIGH};
    for (uint32_t i = 0; i < 2U; i++) {
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(
            hi2c,
            AMG8833_HAL_ADDR(candidates[i]),
            2U,
            AMG8833_I2C_TIMEOUT_MS
        );
        if (st == HAL_OK) {
            *detected_addr_7bit = candidates[i];
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef AMG8833_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st;

    /* PCTL = 0x00: normal mode */
    st = amg8833_write8(hi2c, addr_7bit, AMG8833_REG_PCTL, 0x00U);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    /* RST = 0x3F: initial reset */
    st = amg8833_write8(hi2c, addr_7bit, AMG8833_REG_RST, 0x3FU);
    if (st != HAL_OK) return st;
    HAL_Delay(50);

    /* FPSC = 0x00: 10 FPS output rate */
    st = amg8833_write8(hi2c, addr_7bit, AMG8833_REG_FPSC, 0x00U);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef AMG8833_ReadFrameRaw(
    I2C_HandleTypeDef *hi2c,
    uint8_t addr_7bit,
    uint8_t raw_frame[AMG8833_FRAME_BYTES]
) {
    if ((hi2c == NULL) || (raw_frame == NULL)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        hi2c,
        AMG8833_HAL_ADDR(addr_7bit),
        AMG8833_REG_PIXEL_BASE,
        I2C_MEMADD_SIZE_8BIT,
        raw_frame,
        AMG8833_FRAME_BYTES,
        AMG8833_I2C_READ_TIMEOUT_MS
    );
}

void AMG8833_ConvertRawToCelsius(
    const uint8_t raw_frame[AMG8833_FRAME_BYTES],
    float pixels_c[AMG8833_PIXEL_COUNT]
) {
    if ((raw_frame == NULL) || (pixels_c == NULL)) {
        return;
    }

    for (uint32_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
        uint16_t raw = (uint16_t)raw_frame[2U * i]
                     | ((uint16_t)raw_frame[(2U * i) + 1U] << 8);
        pixels_c[i] = amg8833_raw_pixel_to_c(raw);
    }
}

HAL_StatusTypeDef AMG8833_ReadFrameCelsius(
    I2C_HandleTypeDef *hi2c,
    uint8_t addr_7bit,
    float pixels_c[AMG8833_PIXEL_COUNT]
) {
    uint8_t raw_frame[AMG8833_FRAME_BYTES];
    HAL_StatusTypeDef st = AMG8833_ReadFrameRaw(hi2c, addr_7bit, raw_frame);
    if (st != HAL_OK) {
        return st;
    }

    AMG8833_ConvertRawToCelsius(raw_frame, pixels_c);
    return HAL_OK;
}
