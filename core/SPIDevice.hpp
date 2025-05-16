/* module; */
#pragma once

#include <atomic>
#include <cassert>
#include <coroutine>
#include <cxxutil.h>
#include <memory>
#include <runtime_headers.h>
#include <span>

#include <InplaceAtomicSet.h>
#include <InplaceSet.h>
#include <SpinLock.h>

/* export module core.IO.SerialInterfaces:SPI; */

/* import :Generic; */
#include <SerialInterfaceDevice.hpp>
/* import core.Fundamental; */
#include <cxxutil.hpp>
/* import core.IO.Embedded; */
#include <GPIOManager.hpp>

/* export */ namespace atmc
{
    class SPIDevice;

    class SPIManager final
    {
    public:
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> txDone;
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> rxDone;
        static sys::InplaceAtomicSet<SPI_HandleTypeDef*, Config::SPIBusCount> txrxDone;
        static_assert(std::atomic<SPI_HandleTypeDef*>::is_always_lock_free, "Atomic SPI handle must be lock-free.");

        static sys::SpinLock busyLock;
        static sys::InplaceSet<SPI_HandleTypeDef*, Config::SPIBusCount> busy;

        SPIManager() = delete;

        friend class atmc::SPIDevice;
    };
        
    class SPIDevice final : public SerialInterfaceDevice
    {
        uint8_t (*memRegisterByteFromAddr)(uint16_t, bool);
        HardwareStatus (*waitDeviceReadySync)(uint32_t, uint32_t);
        SPI_HandleTypeDef* internalHandle;
        GPIOPin csPin;
        
        inline sys::Task<HardwareStatus> rwMemory(uint16_t memAddr, sys::Task<HardwareStatus> rw)
        {
            uint8_t _memAddr = this->memRegisterByteFromAddr(memAddr, true);

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
        inline SPIDevice(sys::FencedPointer<SPI_HandleTypeDef> hspi, GPIOPin csPin,
                         uint8_t (*registerByteFromAddr)(uint16_t, bool) = [](uint16_t addr, [[maybe_unused]] bool read) -> uint8_t { return addr; },
                         HardwareStatus (*waitDeviceReadySync)(uint32_t, uint32_t) = []([[maybe_unused]] uint32_t trials, [[maybe_unused]] uint32_t timeout) -> HardwareStatus { return HardwareStatus::Ok; }) :
        memRegisterByteFromAddr(registerByteFromAddr), waitDeviceReadySync(waitDeviceReadySync), internalHandle(&*hspi), csPin(csPin)
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
            
            inline AcquireGuard(sys::FencedPointer<SPIDevice> hspi) : hspi(&*hspi)
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
                    sys::LockGuard guard(SPIManager::busyLock);
                    SPIManager::busy.tryErase(this->hspi);
                }
            }
            
            friend class atmc::SPIDevice;
        private:
            SPI_HandleTypeDef* hspi;
            
            inline AccessGuard(sys::FencedPointer<SPI_HandleTypeDef> hspi) : hspi(&*hspi)
            { }
        };

        sys::Task<AccessGuard> guardBus()
        {
        Retry:
            {
                sys::LockGuard guard(SPIManager::busyLock);
                if (!SPIManager::busy.tryInsert(this->internalHandle))
                {
                    co_await sys::Task<>::yield();
                    goto Retry;
                }
            }
            co_return AccessGuard(this->internalHandle);
        }

        constexpr HardwareStatus waitReadySync([[maybe_unused]] uint32_t trials, [[maybe_unused]] uint32_t timeout = HAL_MAX_DELAY) override
        {
            return HardwareStatus::Ok;
        }

        /// @brief 
        /// @param data 
        /// @return 
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::Task<HardwareStatus> readMemoryUnchecked(std::span<uint8_t> data)
        {
            HardwareStatus res = __sc(HardwareStatus, HAL_SPI_Receive_IT(this->internalHandle, data.data(), data.size_bytes()));
            __fence_value_co_return(res, res != HardwareStatus::Ok);
        
            while (!SPIManager::rxDone.exchange(this->internalHandle, nullptr))
                co_await sys::Task<>::yield();
        
            co_return HardwareStatus::Ok;
        }
        /// @brief 
        /// @param data 
        /// @return 
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::Task<HardwareStatus> writeMemoryUnchecked(std::span<const uint8_t> data)
        {
            HardwareStatus res = __sc(HardwareStatus, HAL_SPI_Transmit_IT(this->internalHandle, data.data(), data.size_bytes()));
            __fence_value_co_return(res, res != HardwareStatus::Ok);
        
            while (!SPIManager::txDone.exchange(this->internalHandle, nullptr))
                co_await sys::Task<>::yield();
        
            co_return HardwareStatus::Ok;
        }
        /// @brief 
        /// @param dataRx 
        /// @param dataTx 
        /// @param dataSize 
        /// @return 
        /// @note This function is marked unchecked because it does not hold the CS line low, nor lock the bus.
        sys::Task<HardwareStatus> exchangeMemoryUnchecked(uint8_t dataRx[], uint8_t dataTx[], size_t dataSize)
        {
            HardwareStatus res = __sc(HardwareStatus, HAL_SPI_TransmitReceive_IT(this->internalHandle, dataRx, dataTx, dataSize));
            __fence_value_co_return(res, res != HardwareStatus::Ok);
        
            while (!SPIManager::txrxDone.exchange(this->internalHandle, nullptr))
                co_await sys::Task<>::yield();
        
            co_return HardwareStatus::Ok;
        }

        inline sys::Task<HardwareStatus> readMemory(std::span<uint8_t> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->readMemoryUnchecked(data);
        }
        inline sys::Task<HardwareStatus> writeMemory(std::span<const uint8_t> data)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->writeMemoryUnchecked(data);
        }
        inline sys::Task<HardwareStatus> exchangeMemory(uint8_t dataRx[], uint8_t dataTx[], size_t dataSize)
        {
            auto accGuard = co_await this->guardBus();
            auto csGuard = this->acquire();
            co_return co_await this->exchangeMemoryUnchecked(dataRx, dataTx, dataSize);
        }

        inline sys::Task<HardwareStatus> readMemory(uint16_t memAddr, std::span<uint8_t> data) override
        {
            return this->rwMemory(memAddr, this->readMemoryUnchecked(data));
        }
        inline sys::Task<HardwareStatus> writeMemory(uint16_t memAddr, std::span<uint8_t> data) override
        {
            return this->rwMemory(memAddr, this->writeMemoryUnchecked(data));
        }

        friend class atmc::SPIManager;
    };
}
