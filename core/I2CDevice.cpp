#include "I2CDevice.h"

// clang-format off
#include <module/sys>
#include <module/sys.Containers>
// clang-format on

#include <Config.h>

using namespace atmc;

inplace_atomic_set<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::txDone(nullptr);
inplace_atomic_set<I2C_HandleTypeDef*, Config::I2CBusCount> I2CManager::rxDone(nullptr);

extern "C" void HAL_I2C_MemTxCpltCallback([[maybe_unused]] I2C_HandleTypeDef* hi2c) { assert(I2CManager::txDone.exchange(nullptr, hi2c) && "whoops"); }
extern "C" void HAL_I2C_MemRxCpltCallback([[maybe_unused]] I2C_HandleTypeDef* hi2c) { assert(I2CManager::rxDone.exchange(nullptr, hi2c) && "whoops"); }
