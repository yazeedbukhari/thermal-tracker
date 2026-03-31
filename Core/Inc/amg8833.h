/*
 * amg8833.h — AMG8833 thermal array sensor driver (STM32 HAL I2C)
 *
 * Brings up the AMG8833, reads full 8x8 raw frames, and converts raw pixel
 * data into Celsius values. This module is hardware-facing only; thresholding,
 * bounding boxes, and centroid logic should live in thermal.c.
 *
 * Public API:  AMG8833_Probe, AMG8833_Init, AMG8833_ReadFrameRaw,
 *              AMG8833_ConvertRawToCelsius, AMG8833_ReadFrameCelsius
 *
 * Owner:
 */

#ifndef INC_AMG8833_H_
#define INC_AMG8833_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit addresses selected by AD_SELECT pin state. */
#define AMG8833_ADDR_7BIT_LOW  0x68U
#define AMG8833_ADDR_7BIT_HIGH 0x69U

/* Register map */
#define AMG8833_REG_PCTL       0x00U
#define AMG8833_REG_RST        0x01U
#define AMG8833_REG_FPSC       0x02U
#define AMG8833_REG_PIXEL_BASE 0x80U

/* Frame geometry */
#define AMG8833_PIXEL_COUNT 64U
#define AMG8833_FRAME_BYTES 128U

/* One byte shifted left for HAL_I2C_* APIs */
#define AMG8833_HAL_ADDR(a7) ((uint16_t)((a7) << 1))

HAL_StatusTypeDef AMG8833_Probe(I2C_HandleTypeDef *hi2c, uint8_t *detected_addr_7bit);
HAL_StatusTypeDef AMG8833_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);

HAL_StatusTypeDef AMG8833_ReadFrameRaw(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit,
                                       uint8_t raw_frame[AMG8833_FRAME_BYTES]);

void AMG8833_ConvertRawToCelsius(const uint8_t raw_frame[AMG8833_FRAME_BYTES],
                                 float pixels_c[AMG8833_PIXEL_COUNT]);

HAL_StatusTypeDef AMG8833_ReadFrameCelsius(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit,
                                           float pixels_c[AMG8833_PIXEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* INC_AMG8833_H_ */
