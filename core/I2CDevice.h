#pragma once

#include <atomic>
#include <cassert>
#include <module/sys>
#include <runtime_headers.h>
#include <span>

#include <Config.h>
#include <SerialInterfaceDevice.h>

namespace atmc
{
#if _libatmc_target_stm32
    using I2CNativeHandle = I2C_HandleTypeDef;
#else
    using I2CNativeHandle = byte;
#endif

    class I2CDevice;

    /// @brief I2C manager.
    /// @note Static class.
    class I2CManager final
    {
    public:
        static sys::inplace_atomic_set<I2C_HandleTypeDef*, atmc::Config::I2CBusCount> txDone;
        static sys::inplace_atomic_set<I2C_HandleTypeDef*, atmc::Config::I2CBusCount> rxDone;
        static_assert(std::atomic<I2C_HandleTypeDef*>::is_always_lock_free, "Atomic I2C handle must be lock-free.");

        I2CManager() = delete;

        friend class atmc::I2CDevice;
    };

    /// @brief I2C device.
    /// @note Pass `byref`.
    class I2CDevice final : public SerialInterfaceDevice
    {
        I2CNativeHandle* internalHandle;
        u16 devAddr;
        u16 memAddrSize;
    public:
        /// @brief Construct an `I2CDevice`.
        /// @param hi2c Underlying system I2C handle.
        /// @param devAddr Device address.
        /// @param memAddrSize Memory address size.
        /// @attention Lifetime assumptions!
        /// ```cpp
        /// (&*hi2c)->decltype(*&*hi2c)(...);
        /// ...
        /// this->I2CDevice(...);
        /// ...
        /// this->~I2CDevice();
        /// ...
        /// (&*hi2c)->~decltype(*&*hi2c)();
        /// ```
        inline I2CDevice(I2C_HandleTypeDef& hi2c, u16 devAddr, u16 memAddrSize) : internalHandle(&hi2c), devAddr(devAddr), memAddrSize(memAddrSize) { }

#if _libatmc_target_stm32
        HardwareStatus waitReadySync(i32 trials, i32 timeout = i32(HAL_MAX_DELAY)) override // NOLINT(google-default-arguments)
#else
        /// @brief Wait for the device to be ready synchronously.
        /// @param trials Number of trials.
        /// @param timeout Timeout.
        /// @return Whether the operation was successful.
        inline HardwareStatus waitReadySync(i32 trials, i32 timeout = i32(HAL_MAX_DELAY)) override
        {
            return HardwareStatus(HAL_I2C_IsDeviceReady(this->internalHandle, *(this->devAddr << 1_u16), sys::numeric_cast<uint32_t>(*trials, unsafe()),
                                                        sys::numeric_cast<uint32_t>(*timeout, unsafe())));
        }

        /// @brief Read memory asynchronously.
        /// @param memAddr Memory address.
        /// @param data Data to read to.
        /// @return Whether the operation was successful.
        /// @attention Lifetime assumptions!
        /// ```cpp
        /// data->decltype(data)(...);
        /// ...
        /// ... = co_await I2CManager::readMemory(...);
        /// ...
        /// data->~decltype(data)();
        /// ```
        sys::task<HardwareStatus> readMemory(u16 memAddr, std::span<byte> data) override
        {
            HardwareStatus res = HardwareStatus(HAL_I2C_Mem_Read_IT(this->internalHandle, *(this->devAddr << 1_u16), *memAddr, *this->memAddrSize, data.data(),
                                                                    sys::numeric_cast<uint16_t>(data.size_bytes(), unsafe())));
            _coretif(res, res != HardwareStatus::Ok);

            while (!I2CManager::rxDone.exchange(this->internalHandle, nullptr))
                co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
#else
            (void)memAddr;
            (void)data;
            co_return HardwareStatus::Ok;
#endif
        }
        /// @brief Write memory asynchronously.
        /// @param memAddr Memory address.
        /// @param data Data to write.
        /// @return Whether the operation was successful.
        /// ```cpp
        /// data->decltype(data)(...);
        /// ...
        /// ... = co_await I2CManager::writeMemory(...);
        /// ...
        /// data->~decltype(data)();
        /// ```
        sys::task<HardwareStatus> writeMemory(u16 memAddr, std::span<byte> data) override
        {
            HardwareStatus res = HardwareStatus(HAL_I2C_Mem_Write_IT(this->internalHandle, *(this->devAddr << 1_u16), *memAddr, *this->memAddrSize, data.data(),
                                                                     sys::numeric_cast<uint16_t>(data.size_bytes(), unsafe())));
            _coretif(res, res != HardwareStatus::Ok);

            while (!I2CManager::txDone.exchange(this->internalHandle, nullptr))
                co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
#else
            (void)memAddr;
            (void)data;
            co_return HardwareStatus::Ok;
#endif
        }

        friend class atmc::I2CManager;
    };
} // namespace atmc
