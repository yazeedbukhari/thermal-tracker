#include "laser.h"

/* External trigger button (active-low with pull-up). */
#define LASER_BTN_GPIO_Port GPIOC
#define LASER_BTN_Pin       GPIO_PIN_2

/* Laser output (active-high for current wiring):
 *   PD12 LOW  -> laser OFF
 *   PD12 HIGH -> laser ON  */
#define LASER_OUT_GPIO_Port GPIOD
#define LASER_OUT_Pin       GPIO_PIN_12

#define LASER_LOCK_DELAY_MS 3000U

static uint32_t g_lock_start_ms = 0U;

static void laser_set(uint8_t on)
{
  HAL_GPIO_WritePin(LASER_OUT_GPIO_Port, LASER_OUT_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Laser_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin = LASER_BTN_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LASER_BTN_GPIO_Port, &gpio);

  g_lock_start_ms = 0U;
  laser_set(0U);
}

void Laser_Update(uint8_t target_locked)
{
  uint32_t now = HAL_GetTick();
  uint8_t button_pressed = (HAL_GPIO_ReadPin(LASER_BTN_GPIO_Port, LASER_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  uint8_t lock_ok = 0U;

  if (target_locked != 0U) {
    if (g_lock_start_ms == 0U) {
      g_lock_start_ms = now;
    }
    if ((now - g_lock_start_ms) >= LASER_LOCK_DELAY_MS) {
      lock_ok = 1U;
    }
  } else {
    g_lock_start_ms = 0U;
  }

  if ((button_pressed != 0U) && (lock_ok != 0U)) {
    laser_set(1U);
  } else {
    laser_set(0U);
  }
}
