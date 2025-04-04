#include <GPIOManager.hpp>
#include <I2CDevice.hpp>
#include <SPIDevice.hpp>

using namespace atmc;

/* export */ extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    int pinIndex = std::bit_width(pin) - 1;
    atmc::GPIOManager::pinFlag[pinIndex].clear();
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
