#pragma once

#include <atomic>
#include <cassert>
#include <runtime_headers.h>
#include <span>

#include <Config.h>
#include <GPIOManager.h>
#include <SerialInterfaceDevice.h>
#include <SpinLock.h>
#include <Target.h>

namespace atmc
{
#if _libatmc_target_stm32
    using SPINativeHandle = SPI_HandleTypeDef;
#else
    using SPINativeHandle = byte;
#endif

    class SPIDevice;

    class SPIManager final
    {
    public:
        static sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> txDone;
        static sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> rxDone;
        static sys::inplace_atomic_set<SPI_HandleTypeDef*, Config::SPIBusCount> txrxDone;
        static_assert(std::atomic<SPI_HandleTypeDef*>::is_always_lock_free, "Atomic SPI handle must be lock-free.");

        static atmc::SpinLock busyLock;
        static sys::inplace_set<SPI_HandleTypeDef*, Config::SPIBusCount> busy;

        SPIManager() = delete;

        friend class atmc::SPIDevice;
    };

    class SPIDevice final : public SerialInterfaceDevice
    {
        u8 (*memRegisterByteFromAddr)(u16, bool);
        HardwareStatus (*waitDeviceReadySync)(u32, u32);
        SPINativeHandle* internalHandle;
        GPIOPin csPin;

        sys::task<HardwareStatus> rwMemory(u16 memAddr, sys::task<HardwareStatus> rw)
        {
            byte _memAddr = *this->memRegisterByteFromAddr(memAddr, true);

            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();

            HardwareStatus res = co_await this->writeMemoryUnchecked(std::span(&mAddr, 1));
            if (res != HardwareStatus::Ok)
                goto Finally;
            res = co_await rw;
        Finally:
            co_return res;
        }
    public:
        SPIDevice(SPI_HandleTypeDef& hspi, GPIOPin csPin, u8 (*registerByteFromAddr)(u16, bool) = [](u16 addr, [[maybe_unused]] bool read) -> u8 { return *addr; },
                  HardwareStatus (*waitDeviceReadySync)(u32, u32) = []([[maybe_unused]] u32 trials, [[maybe_unused]] u32 timeout) -> HardwareStatus
        { return HardwareStatus::Ok; }) : memRegisterByteFromAddr(registerByteFromAddr), waitDeviceReadySync(waitDeviceReadySync), internalHandle(&hspi), csPin(csPin)
        { }

        /// @brief You must call this, on _every_ `atmc::SPIDevice`, before using _any_ SPI functionality.
        void begin() { this->deselect(); }

        void select() { GPIOManager::digitalWrite(this->csPin, GPIOPin::Low); }
        void deselect() { GPIOManager::digitalWrite(this->csPin, GPIOPin::High); }

        struct AcquireGuard
        {
            AcquireGuard(const AcquireGuard&) = delete;
            AcquireGuard(AcquireGuard&& other)
            {
                this->hspi = other.hspi;
                other.hspi = nullptr;
            }
            ~AcquireGuard()
            {
                if (this->hspi)
                    this->hspi->deselect();
            }

            AcquireGuard& operator=(const AcquireGuard&) = delete;
            AcquireGuard& operator=(AcquireGuard&& other) noexcept
            {
                if (this != &other) [[likely]]
                    this->moveFrom(other, unsafe());
                return *this;
            }

            friend class atmc::SPIDevice;
        private:
            SPIDevice* hspi = nullptr;

            AcquireGuard(SPIDevice& hspi) : hspi(&hspi) { this->hspi->select(); }
        };

        AcquireGuard acquire() { return AcquireGuard(*this); }

        struct AccessGuard
        {
            AccessGuard(const AccessGuard&) = delete;
            AccessGuard(AccessGuard&& other)
            {
                this->hspi = other.hspi;
                other.hspi = nullptr;
            }
            ~AccessGuard()
            {
                if (this->hspi)
                {
                    LockGuard guard(SPIManager::busyLock);
                    SPIManager::busy.try_erase(this->hspi);
                }
            }

            AccessGuard& operator=(const AccessGuard&) = delete;
            AccessGuard& operator=(AccessGuard&& other) noexcept
            {
                if (this != &other) [[likely]]
                    this->moveFrom(other, unsafe());
                return *this;
            }

            friend class atmc::SPIDevice;
        private:
            SPINativeHandle* hspi = nullptr;

            AccessGuard(SPI_HandleTypeDef& hspi) : hspi(&hspi) { }
        };

        sys::task<AccessGuard> guardBus()
        {
        Retry:
            {
                LockGuard guard(SPIManager::busyLock);
                if (!SPIManager::busy.try_insert(this->internalHandle))
                {
                    co_await sys::task<>::yield();
                    goto Retry;
                }
            }
            co_return AccessGuard(*this->internalHandle);
        }

        constexpr HardwareStatus waitReadySync([[maybe_unused]] i32 trials, [[maybe_unused]] i32 timeout = i32(HAL_MAX_DELAY)) override { return HardwareStatus::Ok; }

        /// @brief
        /// @param data
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::task<HardwareStatus> readMemoryUnchecked(std::span<byte> data) // NOLINT(readability-convert-member-functions-to-static)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_Receive_IT(this->internalHandle, data.data(), sys::numeric_cast<uint16_t>(data.size_bytes(), unsafe())));
            _coretif(res, res != HardwareStatus::Ok);

            while (!SPIManager::rxDone.exchange(this->internalHandle, nullptr))
                co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
#else
            (void)data;
            co_return HardwareStatus::Ok;
#endif
        }
        /// @brief
        /// @param data
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::task<HardwareStatus> writeMemoryUnchecked(std::span<const byte> data) // NOLINT(readability-convert-member-functions-to-static)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_Transmit_IT(this->internalHandle, data.data(), sys::numeric_cast<uint16_t>(data.size_bytes(), unsafe())));
            _coretif(res, res != HardwareStatus::Ok);

            while (!SPIManager::txDone.exchange(this->internalHandle, nullptr))
                co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
#else
            (void)data;
            co_return HardwareStatus::Ok;
#endif
        }
        /// @brief
        /// @param dataRx
        /// @param dataTx
        /// @param dataSize
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        sys::task<HardwareStatus> exchangeMemoryUnchecked(byte dataRx[] /* NOLINT(readability-non-const-parameter) */, byte dataTx[] /* NOLINT(readability-non-const-parameter) */,
                                                          size_t dataSize)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_TransmitReceive_IT(this->internalHandle, dataRx, dataTx, sys::numeric_cast<uint16_t>(dataSize, unsafe())));
            _coretif(res, res != HardwareStatus::Ok);

            while (!SPIManager::txrxDone.exchange(this->internalHandle, nullptr))
                co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
#else
            (void)dataRx;
            (void)dataTx;
            (void)dataSize;
            co_return HardwareStatus::Ok;
#endif
        }

        sys::task<HardwareStatus> readMemory(std::span<byte> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->readMemoryUnchecked(data);
        }
        sys::task<HardwareStatus> writeMemory(std::span<const byte> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->writeMemoryUnchecked(data);
        }
        sys::task<HardwareStatus> exchangeMemory(byte dataRx[], byte dataTx[], size_t dataSize)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->exchangeMemoryUnchecked(dataRx, dataTx, dataSize);
        }

        sys::task<HardwareStatus> readMemory(u16 memAddr, std::span<byte> data) override { return this->rwMemory(memAddr, this->readMemoryUnchecked(data)); }
        sys::task<HardwareStatus> writeMemory(u16 memAddr, std::span<byte> data) override { return this->rwMemory(memAddr, this->writeMemoryUnchecked(data)); }

        friend class atmc::SPIManager;
    };
} // namespace atmc
