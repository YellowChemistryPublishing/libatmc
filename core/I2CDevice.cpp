#include "I2CDevice.h"

#include <module/sys>
#include <module/sys.Containers>

#include <Config.h>

using namespace atmc;

sys::inplace_atomic_set<I2CNativeHandle*, Config::I2CBusCount> I2CManager::txDone(nullptr);
sys::inplace_atomic_set<I2CNativeHandle*, Config::I2CBusCount> I2CManager::rxDone(nullptr);

extern "C" void /* NOLINT(readability-identifier-naming) */ HAL_I2C_MemTxCpltCallback(I2CNativeHandle* hi2c)
{
    _contract_assert(I2CManager::txDone.exchange(nullptr, hi2c) && "whoops");
}
extern "C" void /* NOLINT(readability-identifier-naming) */ HAL_I2C_MemRxCpltCallback(I2CNativeHandle* hi2c)
{
    _contract_assert(I2CManager::rxDone.exchange(nullptr, hi2c) && "whoops");
}
