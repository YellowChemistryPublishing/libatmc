#pragma once

#include <entry.h>
// clang-format off
#include <module/sys>
#include <module/sys.Threading>
// clang-format on
#include <span>

#include <module/sys>

namespace atmc
{
    /// @brief A generic device communication over a serial interface.
    /// @note Pass `byref`.
    class SerialInterfaceDevice
    {
    public:
        SerialInterfaceDevice() = default;
        SerialInterfaceDevice(const SerialInterfaceDevice&) = delete;
        SerialInterfaceDevice(SerialInterfaceDevice&&) = delete;
        virtual ~SerialInterfaceDevice() = default;

        SerialInterfaceDevice& operator=(const SerialInterfaceDevice&) = delete;
        SerialInterfaceDevice& operator=(SerialInterfaceDevice&&) = delete;

        /// @brief Wait for the device's communication bus to be ready.
        /// @param trials Number of trials.
        /// @param timeout Timeout.
        /// @return Whether the operation was successful.
        virtual HardwareStatus waitReadySync(i32 trials, i32 timeout = sys::task<>::max_delay) = 0; // NOLINT(google-default-arguments)

        /// @brief Read data to a register.
        /// @param memAddr Register address.
        /// @param data Data to read.
        /// @return Whether the operation was successful.
        virtual sys::task<HardwareStatus> readMemory(u16 memAddr, std::span<byte> data) = 0;
        /// @brief Write data from a register.
        /// @param memAddr Register address.
        /// @param data write to read.
        /// @return Whether the operation was successful.
        virtual sys::task<HardwareStatus> writeMemory(u16 memAddr, std::span<byte> data) = 0;

        /// @brief Read a 16-bit unsigned integer from a register.
        /// @param memAddr Register address.
        /// @return The value read.
        sys::task<sys::result<u16, HardwareStatus>> readUInt16LSBFirst(u16 memAddr)
        {
            byte data[2];
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(data, 2));
            _coretif(res, res != HardwareStatus::Ok);
            co_return (u16(data[1]) << 8_u16) | u16(data[0]);
        }
        /// @brief Read a 16-bit signed integer from a register.
        /// @param memAddr Register address.
        /// @return The value read.
        sys::task<sys::result<i16, HardwareStatus>> readInt16LSBFirst(u16 memAddr)
        {
            byte data[2];
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(data, 2));
            _coretif(res, res != HardwareStatus::Ok);
            co_return sys::s16fb2(u8(data[1]), u8(data[0]));
        }

        /// @brief Read contiguous data from a register as a specific type.
        /// @tparam T Type of data.
        /// @param memAddr Register address.
        /// @param dataSize Size of type to read.
        /// @return The value read.
        template <typename T>
        requires (std::is_default_constructible_v<T> && std::is_trivially_copyable_v<T>)
        sys::task<sys::result<T, HardwareStatus>> readMemoryAs(u16 memAddr, u16 dataSize = sizeof(T))
        {
            T ret;
            HardwareStatus res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<byte*>(&ret), +dataSize));
            _coretif(res, res != HardwareStatus::Ok);
            co_return ret;
        }
        /// @brief Write data to a register, then read it back to check.
        /// @tparam DataType Type of data.
        /// @param memAddr Register address.
        /// @param data Data to write.
        /// @param dataSize Size of data.
        /// @return Whether the operation was successful.
        template <typename DataType>
        requires (std::is_default_constructible_v<DataType>)
        sys::task<HardwareStatus> writeMemoryChecked(u16 memAddr, DataType data, u16 dataSize = sizeof(DataType))
        {
            HardwareStatus res = co_await this->writeMemory(memAddr, std::span(reinterpret_cast<byte*>(&data), +dataSize));
            _coretif(res, res != HardwareStatus::Ok);

            DataType checkData;
            res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<byte*>(&checkData), +dataSize));
            _coretif(res, res != HardwareStatus::Ok);
            _coretif(HardwareStatus::Error, data != checkData);

            co_return HardwareStatus::Ok;
        }
        /// @brief Write data to a register, then read it back to check.
        /// @tparam DataType Type of data.
        /// @tparam N Number of elements.
        /// @param memAddr Register address.
        /// @param data Array of values to write.
        /// @return Whether the operation was successful.
        /// @warning Caution the array passed by reference!
        template <typename DataType, size_t N>
        requires (std::is_default_constructible_v<DataType>)
        sys::task<HardwareStatus> writeMemoryChecked(u16 memAddr, DataType (&data)[N]) // NOLINT(cppcoreguidelines-avoid-reference-coroutine-parameters)
        {
            HardwareStatus res = co_await this->writeMemory(memAddr, std::span(reinterpret_cast<byte*>(&data), sizeof(DataType) * N));
            _coretif(res, res != HardwareStatus::Ok);

            DataType checkData[N];
            res = co_await this->readMemory(memAddr, std::span(reinterpret_cast<byte*>(&checkData), sizeof(DataType) * N));
            _coretif(res, res != HardwareStatus::Ok);
            for (size_t i = 0; i < N; i++)
                _coretif(HardwareStatus::Error, data[i] != checkData[i]);

            co_return HardwareStatus::Ok;
        }
    };
} // namespace atmc
