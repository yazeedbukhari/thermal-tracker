/* Thermal display output on the ST7735 panel. */
#ifndef INC_DISPLAY_ST7735_H_
#define INC_DISPLAY_ST7735_H_

#include "main.h"
#include "thermal.h"

void DisplayST7735_Init(void);
void DisplayST7735_DrawGuide(void);
void DisplayST7735_RenderFrame32x32(const float *upscaled_32x32,
                                    const ThermalObjectsResult *objs, uint8_t selected_idx);
void DisplayST7735_GetStats(uint32_t *queued, uint32_t *dropped);

#endif /* INC_DISPLAY_ST7735_H_ */



