#include "SPIDevice.h"

#include <module/sys>
#include <module/sys.Containers>

#include <Config.h>
#include <SpinLock.h>

using namespace atmc;

sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::txDone(nullptr);
sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
sys::inplace_atomic_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
sys::inplace_set<SPINativeHandle*, Config::SPIBusCount> SPIManager::busy;

extern "C" void /* NOLINT(readability-identifier-naming) */ HAL_SPI_TxCpltCallback(SPINativeHandle* hspi)
{
    _contract_assert(SPIManager::txDone.exchange(nullptr, hspi), "whoops");
}
extern "C" void /* NOLINT(readability-identifier-naming) */ HAL_SPI_RxCpltCallback(SPINativeHandle* hspi)
{
    _contract_assert(SPIManager::rxDone.exchange(nullptr, hspi), "whoops");
}
extern "C" void /* NOLINT(readability-identifier-naming) */ HAL_SPI_TxRxCpltCallback(SPINativeHandle* hspi)
{
    _contract_assert(SPIManager::txrxDone.exchange(nullptr, hspi), "whoops");
}
