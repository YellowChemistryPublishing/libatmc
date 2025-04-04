#include <GPIOManager.hpp>
#include <I2CDevice.hpp>
#include <SPIDevice.hpp>
#include <Task.hpp>

#include <FreeRTOS.h>
#include <task.h>

using namespace atmc;

std::atomic_flag GPIOManager::pinFlag[Config::PinCountGPIO];

InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::txDone(nullptr);
InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::rxDone(nullptr);

InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::busy;

template <> uint_least8_t TaskPromiseCore<void>::memAvail[Config::TaskCoroutineBlockSize];
template <> size_t TaskPromiseCore<void>::memAvailEnd = 0;

void* operator new (size_t sz)
{
    void* ret = pvPortMalloc(sz);
    if (!ret)
        throw std::bad_alloc();
    else return ret;
}
void operator delete (void* p) noexcept
{
    vPortFree(p);
}
void operator delete (void* p, size_t) noexcept
{
    vPortFree(p);
}
