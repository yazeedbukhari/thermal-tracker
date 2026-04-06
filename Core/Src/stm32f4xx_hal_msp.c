/* Low-level MCU support package setup for HAL peripherals. */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include "config.h"

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
/* USER CODE END Define */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */
/* USER CODE END Macro */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */
/* USER CODE END ExternalFunctions */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
  void HAL_MspInit(void)
{

    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}


  void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        
          GPIO_InitStruct.Pin       = STLK_RX_Pin | STLK_TX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}


  void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        
          HAL_GPIO_DeInit(GPIOD, STLK_RX_Pin | STLK_TX_Pin);

        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* I2C1 GPIO Configuration
       PB8  ------> I2C1_SCL
       PB9  ------> I2C1_SDA
     */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    }
}


  void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct             = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (hpcd->Instance == USB_OTG_FS) {
        /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

        /* USER CODE END USB_OTG_FS_MspInit 0 */

        
          PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
        PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48CLKSOURCE_PLLQ;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        
          GPIO_InitStruct.Pin       = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = USB_VBUS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

        /* USER CODE END USB_OTG_FS_MspInit 1 */
    }
}


  void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_OTG_FS) {
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

        /* USER CODE END USB_OTG_FS_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

        
          HAL_GPIO_DeInit(GPIOA, USB_SOF_Pin | USB_VBUS_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin);

        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

        /* USER CODE END USB_OTG_FS_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI3) {
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* SPI3 GPIO Configuration
           PB3  ------> SPI3_SCK
           PB5  ------> SPI3_MOSI
         */
        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        __HAL_RCC_SPI3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_5);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* PA3 - VRx (ADC1_IN3) */
        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PC0 - VRy (ADC1_IN10) */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

/* USER CODE END 1 */
