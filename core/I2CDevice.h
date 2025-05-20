#pragma once

#include <atomic>
#include <cassert>
#include <runtime_headers.h>
#include <span>

#include <InplaceAtomicSet.h>
#include <Result.h>
#include <SerialInterfaceDevice.h>
#include <SpinLock.h>

namespace atmc
{
    class I2CDevice;

    /// @brief I2C manager.
    /// @note Static class.
    class I2CManager final
    {
    public:
        static sys::InplaceAtomicSet<I2C_HandleTypeDef*, atmc::Config::I2CBusCount> txDone;
        static sys::InplaceAtomicSet<I2C_HandleTypeDef*, atmc::Config::I2CBusCount> rxDone;
        static_assert(std::atomic<I2C_HandleTypeDef*>::is_always_lock_free, "Atomic I2C handle must be lock-free.");

        I2CManager() = delete;

        friend class atmc::I2CDevice;
    };

    /// @brief I2C device.
    /// @note Pass `byref`.
    class I2CDevice final : public SerialInterfaceDevice
    {
        I2C_HandleTypeDef* internalHandle;
        uint16_t devAddr;
        uint16_t memAddrSize;
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
        inline I2CDevice(sys::FencedPointer<I2C_HandleTypeDef> hi2c, uint16_t devAddr, uint16_t memAddrSize) :
            internalHandle(&*hi2c /* Contract implied: `hi2c != nullptr`. */), devAddr(devAddr), memAddrSize(memAddrSize)
        { }

        /// @brief Wait for the device to be ready synchronously.
        /// @param trials Number of trials.
        /// @param timeout Timeout.
        /// @return Whether the operation was successful.
        inline HardwareStatus waitReadySync(uint32_t trials, uint32_t timeout = HAL_MAX_DELAY) override
        {
            return HardwareStatus(HAL_I2C_IsDeviceReady(this->internalHandle, this->devAddr << 1, trials, timeout));
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
        sys::Task<HardwareStatus> readMemory(uint16_t memAddr, std::span<uint8_t> data) override
        {
            HardwareStatus res = HardwareStatus(HAL_I2C_Mem_Read_IT(this->internalHandle, this->devAddr << 1, memAddr, this->memAddrSize, data.data(), data.size_bytes()));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            while (!I2CManager::rxDone.exchange(this->internalHandle, nullptr)) co_await sys::Task<>::yield();

            co_return HardwareStatus::Ok;
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
        sys::Task<HardwareStatus> writeMemory(uint16_t memAddr, std::span<uint8_t> data) override
        {
            HardwareStatus res = HardwareStatus(HAL_I2C_Mem_Write_IT(this->internalHandle, this->devAddr << 1, memAddr, this->memAddrSize, data.data(), data.size_bytes()));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            while (!I2CManager::txDone.exchange(this->internalHandle, nullptr)) co_await sys::Task<>::yield();

            co_return HardwareStatus::Ok;
        }

        friend class atmc::I2CManager;
    };
} // namespace atmc
