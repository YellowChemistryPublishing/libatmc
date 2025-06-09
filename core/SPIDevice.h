#pragma once

#include <atomic>
#include <cassert>
#include <numeric>
#include <runtime_headers.h>
#include <span>

#include <GPIOManager.h>
#include <InplaceAtomicSet.h>
#include <InplaceSet.h>
#include <LanguageSupport.h>
#include <SerialInterfaceDevice.h>
#include <SpinLock.h>

namespace atmc
{
    class SPIDevice;

    class SPIManager final
    {
    public:
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> txDone;
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> rxDone;
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> txrxDone;
        static_assert(std::atomic<SPI_HandleTypeDef*>::is_always_lock_free, "Atomic SPI handle must be lock-free.");

        static sys::spin_lock busyLock;
        static sys::InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> busy;

        SPIManager() = delete;

        friend class atmc::SPIDevice;
    };

    class SPIDevice final : public SerialInterfaceDevice
    {
        u8 (*memRegisterByteFromAddr)(u16, bool);
        HardwareStatus (*waitDeviceReadySync)(u32, u32);
        SPI_HandleTypeDef* internalHandle;
        GPIOPin csPin;

        inline sys::task<HardwareStatus> rwMemory(u16 memAddr, sys::task<HardwareStatus> rw)
        {
            byte _memAddr = +this->memRegisterByteFromAddr(memAddr, true);

            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();

            HardwareStatus res = co_await this->writeMemoryUnchecked(std::span(&_memAddr, 1));
            if (res != HardwareStatus::Ok)
                goto Finally;
            res = co_await rw;
        Finally:
            co_return res;
        }
    public:
        inline SPIDevice(sys::fenced_pointer<SPI_HandleTypeDef> hspi, GPIOPin csPin, u8 (*registerByteFromAddr)(u16, bool) = [](u16 addr, [[maybe_unused]] bool read) -> u8
        { return +addr; }, HardwareStatus (*waitDeviceReadySync)(u32, u32) = []([[maybe_unused]] u32 trials, [[maybe_unused]] u32 timeout) -> HardwareStatus
        { return HardwareStatus::Ok; }) : memRegisterByteFromAddr(registerByteFromAddr), waitDeviceReadySync(waitDeviceReadySync), internalHandle(&*hspi), csPin(csPin)
        { }

        /// @brief You must call this, on _every_ `atmc::SPIDevice`, before using _any_ SPI functionality.
        inline void begin()
        {
            this->deselect();
        }

        inline void select()
        {
            GPIOManager::digitalWrite(this->csPin, GPIOPin::Low);
        }
        inline void deselect()
        {
            GPIOManager::digitalWrite(this->csPin, GPIOPin::High);
        }

        struct AcquireGuard
        {
            inline AcquireGuard(const AcquireGuard&) = delete;
            inline AcquireGuard(AcquireGuard&& other)
            {
                this->hspi = other.hspi;
                other.hspi = nullptr;
            }
            inline ~AcquireGuard()
            {
                if (this->hspi)
                    this->hspi->deselect();
            }

            friend class atmc::SPIDevice;
        private:
            SPIDevice* hspi;

            inline AcquireGuard(sys::fenced_pointer<SPIDevice> hspi) : hspi(&*hspi)
            {
                this->hspi->select();
            }
        };

        inline AcquireGuard acquire()
        {
            return AcquireGuard(this);
        }

        struct AccessGuard
        {
            inline AccessGuard(const AccessGuard&) = delete;
            inline AccessGuard(AccessGuard&& other)
            {
                this->hspi = other.hspi;
                other.hspi = nullptr;
            }
            inline ~AccessGuard()
            {
                if (this->hspi)
                {
                    sys::lock_guard guard(SPIManager::busyLock);
                    SPIManager::busy.tryErase(this->hspi);
                }
            }

            friend class atmc::SPIDevice;
        private:
            SPI_HandleTypeDef* hspi;

            inline AccessGuard(sys::fenced_pointer<SPI_HandleTypeDef> hspi) : hspi(&*hspi)
            { }
        };

        sys::task<AccessGuard> guardBus()
        {
        Retry:
            {
                sys::lock_guard guard(SPIManager::busyLock);
                if (!SPIManager::busy.tryInsert(this->internalHandle))
                {
                    co_await sys::task<>::yield();
                    goto Retry;
                }
            }
            co_return AccessGuard(this->internalHandle);
        }

        constexpr HardwareStatus waitReadySync([[maybe_unused]] i32 trials, [[maybe_unused]] i32 timeout = HAL_MAX_DELAY) override
        {
            return HardwareStatus::Ok;
        }

        /// @brief
        /// @param data
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::task<HardwareStatus> readMemoryUnchecked(std::span<byte> data)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_Receive_IT(this->internalHandle, data.data(), std::saturate_cast<uint16_t>(data.size_bytes())));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            while (!SPIManager::rxDone.exchange(this->internalHandle, nullptr)) co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
        }
        /// @brief
        /// @param data
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::task<HardwareStatus> writeMemoryUnchecked(std::span<const byte> data)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_Transmit_IT(this->internalHandle, data.data(), std::saturate_cast<uint16_t>(data.size_bytes())));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            while (!SPIManager::txDone.exchange(this->internalHandle, nullptr)) co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
        }
        /// @brief
        /// @param dataRx
        /// @param dataTx
        /// @param dataSize
        /// @return
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::task<HardwareStatus> exchangeMemoryUnchecked(byte dataRx[], byte dataTx[], size_t dataSize)
        {
            HardwareStatus res = HardwareStatus(HAL_SPI_TransmitReceive_IT(this->internalHandle, dataRx, dataTx, std::saturate_cast<uint16_t>(dataSize)));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            while (!SPIManager::txrxDone.exchange(this->internalHandle, nullptr)) co_await sys::task<>::yield();

            co_return HardwareStatus::Ok;
        }

        inline sys::task<HardwareStatus> readMemory(std::span<byte> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->readMemoryUnchecked(data);
        }
        inline sys::task<HardwareStatus> writeMemory(std::span<const byte> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->writeMemoryUnchecked(data);
        }
        inline sys::task<HardwareStatus> exchangeMemory(byte dataRx[], byte dataTx[], size_t dataSize)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->exchangeMemoryUnchecked(dataRx, dataTx, dataSize);
        }

        inline sys::task<HardwareStatus> readMemory(u16 memAddr, std::span<byte> data) override
        {
            return this->rwMemory(memAddr, this->readMemoryUnchecked(data));
        }
        inline sys::task<HardwareStatus> writeMemory(u16 memAddr, std::span<byte> data) override
        {
            return this->rwMemory(memAddr, this->writeMemoryUnchecked(data));
        }

        friend class atmc::SPIManager;
    };
} // namespace atmc
