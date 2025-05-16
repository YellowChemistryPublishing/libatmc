#include "I2CDevice.hpp"

using namespace sys;
using namespace atmc;

InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::txDone(nullptr);
InplaceAtomicSet<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::rxDone(nullptr);
