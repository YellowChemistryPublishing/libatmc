#include <GPIOManager.hpp>
#include <I2CDevice.hpp>
#include <SPIDevice.hpp>

using namespace atmc;

/* export */ extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    int pinIndex = std::bit_width(pin) - 1;
    atmc::GPIOManager::pinFlag[pinIndex].clear();
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
    assert(I2CManager::txDone.exchange(nullptr, hi2c) && "whoops");
}
/* export */ extern "C" void HAL_I2C_MemRxCpltCallback([[maybe_unused]] I2C_HandleTypeDef* hi2c)
{
    assert(I2CManager::rxDone.exchange(nullptr, hi2c) && "whoops");
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
