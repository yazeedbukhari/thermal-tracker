#ifndef INC_ST7735_CN8_SPI_TEST_H_
#define INC_ST7735_CN8_SPI_TEST_H_

#include "main.h"
#include "thermal.h"

void ST7735_CN8_Test_Init(void);
void ST7735_CN8_Test_DrawBox(void);
void ST7735_CN8_RenderFrame32x32(const float *upscaled_32x32, const ThermalObjectsResult *objs,
                                 uint8_t selected_idx);
void ST7735_CN8_GetRenderStats(uint32_t *queued, uint32_t *dropped);

#endif /* INC_ST7735_CN8_SPI_TEST_H_ */
