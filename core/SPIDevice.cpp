#include "SPIDevice.h"

// clang-format off
#include <module/sys>
#include <module/sys.Containers>
// clang-format on

#include <Config.h>
#include <SpinLock.h>

using namespace atmc;

sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::txDone(nullptr);
sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
sys::inplace_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::busy;

extern "C" void HAL_SPI_TxCpltCallback(SPINativeHandle* hspi) // NOLINT(readability-identifier-naming)
{
    _contract_assert(SPIManager::txDone.exchange(nullptr, hspi) && "whoops");
}
extern "C" void HAL_SPI_RxCpltCallback(SPINativeHandle* hspi) // NOLINT(readability-identifier-naming)
{
    _contract_assert(SPIManager::rxDone.exchange(nullptr, hspi) && "whoops");
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPINativeHandle* hspi) // NOLINT(readability-identifier-naming)
{
    _contract_assert(SPIManager::txrxDone.exchange(nullptr, hspi) && "whoops");
}
