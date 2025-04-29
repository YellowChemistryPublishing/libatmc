#include "board.h"

#include <print>

#include <FreeRTOS.h>
#include <task.h>

__weak I2C_HandleTypeDef hi2c1;
__weak I2C_HandleTypeDef hi2c2;
__weak I2C_HandleTypeDef hi2c3;

__weak SPI_HandleTypeDef hspi1;
__weak SPI_HandleTypeDef hspi2;
__weak SPI_HandleTypeDef hspi3;
__weak SPI_HandleTypeDef hspi4;

__weak UART_HandleTypeDef huart1;
__weak UART_HandleTypeDef huart2;
__weak UART_HandleTypeDef huart3;

__weak TIM_HandleTypeDef htim1;
__weak TIM_HandleTypeDef htim2;
__weak TIM_HandleTypeDef htim3;
__weak TIM_HandleTypeDef htim4;
__weak TIM_HandleTypeDef htim5;
__weak TIM_HandleTypeDef htim6;
__weak TIM_HandleTypeDef htim7;
__weak TIM_HandleTypeDef htim8;
__weak TIM_HandleTypeDef htim12;
__weak TIM_HandleTypeDef htim13;
__weak TIM_HandleTypeDef htim14;
__weak TIM_HandleTypeDef htim15;
__weak TIM_HandleTypeDef htim16;
__weak TIM_HandleTypeDef htim17;

__weak ADC_HandleTypeDef hadc1;
__weak DMA_HandleTypeDef hdma_adc1;
__weak ADC_HandleTypeDef hadc2;
__weak DMA_HandleTypeDef hdma_adc2;
__weak ADC_HandleTypeDef hadc3;
__weak DMA_HandleTypeDef hdma_adc3;

extern "C" void xPortSysTickHandler();

extern "C" void HardFault_Handler()
{
    std::println("Hardfault!");
    while (true);
}
extern "C" void SysTick_Handler()
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) [[likely]]
        xPortSysTickHandler();
}

#ifdef STM32H753xx
extern "C" int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
