#include "SPIDevice.hpp"

using namespace atmc;

InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::rxDone(nullptr);
InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::txrxDone(nullptr);

SpinLock SPIManager::busyLock;
InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> SPIManager::busy;
