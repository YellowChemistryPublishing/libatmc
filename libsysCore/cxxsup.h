#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>

#include <Result.h>

namespace sys
{
    template <typename T, typename U = void>
    concept IEnumerable = std::is_array_v<T> || requires(T range) {
        range.begin();
        range.end();

        range.begin() != range.end();
        ++range.begin();

        requires ((std::same_as<U, void> && requires { *range.begin(); }) || std::convertible_to<decltype(*range.begin()), U&>);
    } || requires(T range) {
        begin(range);
        end(range);

        begin(range) != end(range);
        ++begin(range);

        requires ((std::same_as<U, void> && requires { *begin(range); }) || std::convertible_to<decltype(*begin(range)), U&>);
    };

    constexpr i32 nr2i32(i32 v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }
    constexpr i64 nr2i64(i64 v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v |= v >> 32;
        v++;
        return v;
    }

    /// @brief Obtain the two's complement signed 16-bit integer from two bytes.
    /// @param msb The most significant byte.
    /// @param lsb The least significant byte.
    /// @return Signed 16-bit integer.
    constexpr int16_t s16fb2(uint8_t msb, uint8_t lsb)
    {
        return (int16_t(msb) << 8) | int16_t(lsb);
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
    template <std::integral T, std::floating_point ValueType>
    constexpr Result<T> boundedCast(ValueType value)
    {
        if (std::numeric_limits<T>::lowest() <= value && value <= std::numeric_limits<T>::max()) [[likely]]
            return T(value);
        else
            return nullptr;
    }
} // namespace sys
