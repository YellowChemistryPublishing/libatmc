#include <GPIOManager.hpp>
#include <I2CDevice.hpp>
#include <SPIDevice.hpp>

using namespace atmc;

/* export */ extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    int pinIndex = std::bit_width(pin) - 1;
    atmc::GPIOManager::pinFlag[pinIndex].clear();
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitTypeDef));
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    memset(&PeriphClkInitStruct, 0, sizeof(RCC_PeriphCLKInitTypeDef));
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 1;
    PeriphClkInitStruct.PLL2.PLL2N = 19;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0.0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_INP16
    PB1     ------> ADC1_INP5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    GPIOManager::hdmaADC[0].Instance = DMA1_Stream0;
    GPIOManager::hdmaADC[0].Init.Request = DMA_REQUEST_ADC1;
    GPIOManager::hdmaADC[0].Init.Direction = DMA_PERIPH_TO_MEMORY;
    GPIOManager::hdmaADC[0].Init.PeriphInc = DMA_PINC_DISABLE;
    GPIOManager::hdmaADC[0].Init.MemInc = DMA_MINC_ENABLE;
    GPIOManager::hdmaADC[0].Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    GPIOManager::hdmaADC[0].Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    GPIOManager::hdmaADC[0].Init.Mode = DMA_CIRCULAR;
    GPIOManager::hdmaADC[0].Init.Priority = DMA_PRIORITY_LOW;
    GPIOManager::hdmaADC[0].Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&GPIOManager::hdmaADC[0]) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,GPIOManager::hdmaADC[0]);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */

  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_INP16
    PB1     ------> ADC1_INP5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* ADC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }

}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
extern "C" void DMA1_Stream0_IRQHandler()
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&GPIOManager::hdmaADC[0]);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}
/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
extern "C" void ADC_IRQHandler()
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&GPIOManager::hadc[0]);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop_DMA(hadc);
    switch (__reic(uintptr_t, hadc->Instance))
    {
    case ADC1_BASE:
        GPIOManager::adcFlags[0].clear();
        break;
    case ADC2_BASE:
        GPIOManager::adcFlags[1].clear();
        break;
    case ADC3_BASE:
        GPIOManager::adcFlags[2].clear();
        break;
    }
}

/* export */ extern "C" void HAL_I2C_MemTxCpltCallback([[maybe_unused]] I2C_HandleTypeDef* hi2c)
{
    if (I2CManager::txDone.exchange(nullptr, hi2c))
        return;
    assert(false && "whoops");
}
/* export */ extern "C" void HAL_I2C_MemRxCpltCallback([[maybe_unused]] I2C_HandleTypeDef* hi2c)
{
    if (I2CManager::rxDone.exchange(nullptr, hi2c))
        return;
    assert(false && "whoops");
}

/* export */ extern "C" void HAL_SPI_TxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi)
{
    assert(SPIManager::txDone.exchange(nullptr, hspi) && "whoops");
}
/* export */ extern "C" void HAL_SPI_RxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi)
{
    assert(SPIManager::rxDone.exchange(nullptr, hspi) && "whoops");
}
/* export */ extern "C" void HAL_SPI_TxRxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi)
{
    assert(SPIManager::txrxDone.exchange(nullptr, hspi) && "whoops");
}
