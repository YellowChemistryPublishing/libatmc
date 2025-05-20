#pragma once

#include <limits>
#include <utility>

#include <LanguageSupport.h>
#include <Result.h>

namespace sys
{
    template <INumber T, INumber ValueType>
    constexpr T numeric_cast(ValueType value, unsafe_tag)
    {
        if (std::cmp_less_equal(std::numeric_limits<T>::lowest(), value) && std::cmp_less_equal(value, std::numeric_limits<T>::max())) [[likely]]
            return T(value);
        else if (std::cmp_less(value, std::numeric_limits<T>::lowest()))
            return std::numeric_limits<T>::lowest();
        else // if (std::cmp_greater(value, std::numeric_limits<T>::max()))
            return std::numeric_limits<T>::max();
    }
    template <INumber T, INumber ValueType>
    constexpr Result<T> numeric_cast(ValueType value)
    {
        if (std::cmp_less_equal(std::numeric_limits<T>::lowest(), value) && std::cmp_less_equal(value, std::numeric_limits<T>::max())) [[likely]]
            return T(value);
        else
            return nullptr;
    }

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
} // namespace sys
