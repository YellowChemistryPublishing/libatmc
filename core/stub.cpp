#include <GPIOManager.hpp>
#include <I2CDevice.hpp>
#include <SPIDevice.hpp>
#include <Task.hpp>

#include <FreeRTOS.h>
#include <task.h>

using namespace atmc;

std::atomic_flag GPIOManager::pinFlag[Config::PinCountGPIO];

std::atomic_flag GPIOManager::adcFlags[Config::AnalogConverterCount];
__dma_rw volatile uint16_t GPIOManager::adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels];

InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::txDone(nullptr);
InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::rxDone(nullptr);

InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::busy;

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
