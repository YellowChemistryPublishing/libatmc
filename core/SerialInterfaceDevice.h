#pragma once

#include <cstdint>
#include <cxxsup.h>
#include <entry.h>
#include <span>

#include <Result.h>
#include <TaskEx.h>

namespace atmc
{
    /// @brief A generic device communication over a serial interface.
    /// @note Pass `byref`.
    class SerialInterfaceDevice
    {
    public:
        /// @brief Wait for the device's communication bus to be ready.
        /// @param trials Number of trials.
        /// @param timeout Timeout.
        /// @return Whether the operation was successful.
        virtual HardwareStatus waitReadySync(uint32_t trials, uint32_t timeout = sys::Task<>::MaxDelay) = 0;

        /// @brief Read data to a register.
        /// @param memAddr Register address.
        /// @param data Data to read.
        /// @return Whether the operation was successful.
        virtual sys::Task<HardwareStatus> readMemory(uint16_t memAddr, std::span<uint8_t> data) = 0;
        /// @brief Write data from a register.
        /// @param memAddr Register address.
        /// @param data write to read.
        /// @return Whether the operation was successful.
        virtual sys::Task<HardwareStatus> writeMemory(uint16_t memAddr, std::span<uint8_t> data) = 0;

        /// @brief Read a 16-bit unsigned integer from a register.
        /// @param memAddr Register address.
        /// @return The value read.
        inline sys::Task<sys::Result<uint16_t, HardwareStatus>> readUInt16LSBFirst(uint16_t memAddr)
        {
            uint8_t data[2];
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(data, 2));
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return (uint16_t(data[1]) << 8) | data[0];
        }
        /// @brief Read a 16-bit signed integer from a register.
        /// @param memAddr Register address.
        /// @return The value read.
        inline sys::Task<sys::Result<int16_t, HardwareStatus>> readInt16LSBFirst(uint16_t memAddr)
        {
            uint8_t data[2];
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(data, 2));
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sys::s16fb2(data[1], data[0]);
        }

        /// @brief Read contiguous data from a register as a specific type.
        /// @tparam T Type of data.
        /// @param memAddr Register address.
        /// @param dataSize Size of type to read.
        /// @return The value read.
        template <typename T>
        requires (std::is_default_constructible<T>::value && std::is_trivially_copyable<T>::value)
        inline sys::Task<sys::Result<T, HardwareStatus>> readMemoryAs(uint16_t memAddr, uint16_t dataSize = sizeof(T))
        {
            T ret;
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<uint8_t*>(&ret), dataSize));
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return ret;
        }
        /// @brief Write data to a register, then read it back to check.
        /// @tparam DataType Type of data.
        /// @param memAddr Register address.
        /// @param data Data to write.
        /// @param dataSize Size of data.
        /// @return Whether the operation was successful.
        template <typename DataType>
        requires (std::is_default_constructible<DataType>::value)
        inline sys::Task<HardwareStatus> writeMemoryChecked(uint16_t memAddr, DataType data, uint16_t dataSize = sizeof(DataType))
        {
            HardwareStatus res = co_await this->writeMemory(memAddr, std::span(reinterpret_cast<uint8_t*>(&data), dataSize));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            DataType checkData;
            res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<uint8_t*>(&checkData), dataSize));
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            _fence_value_co_return(HardwareStatus::Error, data != checkData);

            co_return HardwareStatus::Ok;
        }
        /// @brief Write data to a register, then read it back to check.
        /// @tparam DataType Type of data.
        /// @tparam N Number of elements.
        /// @param memAddr Register address.
        /// @param data Array of values to write.
        /// @return Whether the operation was successful.
        template <typename DataType, size_t N>
        requires (std::is_default_constructible<DataType>::value)
        inline sys::Task<HardwareStatus> writeMemoryChecked(uint16_t memAddr, DataType (&data)[N])
        {
            HardwareStatus res = co_await this->writeMemory(memAddr, std::span(reinterpret_cast<uint8_t*>(&data), sizeof(DataType) * N));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            DataType checkData[N];
            res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<uint8_t*>(&checkData), sizeof(DataType) * N));
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            for (size_t i = 0; i < N; i++) _fence_value_co_return(HardwareStatus::Error, data[i] != checkData[i]);

            co_return HardwareStatus::Ok;
        }
    };
} // namespace atmc