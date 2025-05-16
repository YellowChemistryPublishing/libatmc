#include "SPIDevice.hpp"

using namespace atmc;

sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txDone(nullptr);
sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

sys::SpinLock SPIManager::busyLock;
sys::InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::busy;
