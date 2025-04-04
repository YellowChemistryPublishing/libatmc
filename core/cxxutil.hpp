/* module; */
#pragma once

#include <cstdint>
#include <cxxabi.h>
#include <cxxutil.h>
#include <exception>
#include <memory>
#include <print>
#include <runtime_headers.h>
#include <utility>

#include <FreeRTOS.h>
#include <task.h>

#include <CompilerWarnings.h>

/* export module core.Fundamental; */

/* export */ namespace atmc
{
	/// @brief The status of a hardware operation.
	enum class HardwareStatus : uint_least8_t
	{
		Ok = HAL_OK,
		Error = HAL_ERROR,
		Busy = HAL_BUSY,
		Timeout = HAL_TIMEOUT
	};

	/// @brief Obtain the two's complement signed 16-bit integer from two bytes.
	/// @param msb The most significant byte.
	/// @param lsb The least significant byte.
	/// @return Signed 16-bit integer.
	constexpr int16_t s16fb2(uint8_t msb, uint8_t lsb)
	{
		return (int16_t(msb) << 8) | (int16_t)lsb;
	}
	/// @brief Obtain the high byte from a signed 16-bit integer.
	/// @param val The signed 16-bit integer.
	/// @return The high byte.
	constexpr uint8_t hbfs16(int16_t val)
	{
		return uint8_t(val >> 8);
	}
    /// @brief Obtain the low byte from a signed 16-bit integer.
    /// @param val The signed 16-bit integer.
    /// @return The low byte.
    constexpr uint8_t lbfs16(int16_t val)
    {
        return uint8_t(val);
    }
	
	/// @brief Convert a floating-point value to an integral value, bounded by the integral type's limits.
	/// @tparam T The integral type to convert to.
	/// @tparam ValueType The floating-point type to convert from.
	/// @param value The value to convert.
	/// @return The bounded integral value.
	template <std::integral T, std::floating_point ValueType = float>
	constexpr T boundedCast(ValueType value)
	{
		if (value > std::numeric_limits<T>::max())
			return std::numeric_limits<T>::max();
		else if (value < std::numeric_limits<T>::lowest())
			return std::numeric_limits<T>::lowest();
		else return T(value);
	}

    struct ThreadCriticalSection
    {
        [[gnu::forceinline]] inline ThreadCriticalSection()
        {
            taskENTER_CRITICAL();
        }
        [[gnu::forceinline]] inline ~ThreadCriticalSection()
        {
            taskEXIT_CRITICAL();
        }
    };
}
