/* ADC-DMA joystick sampling and conversion to normalized axes. */
// VRx uses Pin PA3 (A0)
// VRy uses Pin PC0 (A1)

#include "joystick.h"
#include "config.h"

#define ADC_CENTER 2047.5f
#define DEAD_ZONE  0.10f /* 10% of full scale each side */

static volatile uint16_t adc_dma_buf[2]; /* [0]=VRx, [1]=VRy, updated by DMA */

static float normalize(uint16_t raw)
{
    float norm = ((float)raw - ADC_CENTER) / ADC_CENTER; /* -1.0..+1.0 */
    if (norm > -DEAD_ZONE && norm < DEAD_ZONE)
        return 0.0f;
    if (norm > 0.0f)
        return (norm - DEAD_ZONE) / (1.0f - DEAD_ZONE);
    return (norm + DEAD_ZONE) / (1.0f - DEAD_ZONE);
}

void Joystick_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, 2);
}

void Joystick_DeInit(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
}

JoystickReading read_joystick_adc(void)
{
    JoystickReading r;
    r.vr_x = normalize(adc_dma_buf[0]);
    r.vr_y = normalize(adc_dma_buf[1]);
    return r;
}




