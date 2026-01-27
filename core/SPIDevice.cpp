#include "SPIDevice.h"

#include <SpinLock.h>

using namespace atmc;

sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txDone(nullptr);
sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
sys::inplace_set<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::busy;

extern "C" void HAL_SPI_TxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi) { assert(SPIManager::txDone.exchange(nullptr, hspi) && "whoops"); }
extern "C" void HAL_SPI_RxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi) { assert(SPIManager::rxDone.exchange(nullptr, hspi) && "whoops"); }
extern "C" void HAL_SPI_TxRxCpltCallback([[maybe_unused]] SPI_HandleTypeDef* hspi) { assert(SPIManager::txrxDone.exchange(nullptr, hspi) && "whoops"); }
