#pragma once

#include <cmath>

namespace atmc
{
    #define __implement_unit_universal(T, ScalarType) \
 \
    constexpr explicit T(ScalarType value) noexcept : value(value) { } \
 \
    friend constexpr auto operator<=>(const T&, const T&) noexcept = default; \
 \
    friend constexpr T operator+(T lhs, T rhs) noexcept \
    { \
        return T(lhs.value + rhs.value); \
    } \
    friend constexpr T operator-(T lhs, T rhs) noexcept \
    { \
        return T(lhs.value - rhs.value); \
    } \
    friend constexpr T operator*(T lhs, ScalarType rhs) noexcept \
    { \
        return T(lhs.value * rhs); \
    } \
    friend constexpr T operator*(ScalarType lhs, T rhs) noexcept \
    { \
        return T(lhs * rhs.value); \
    } \
    friend constexpr T operator/(T lhs, ScalarType rhs) noexcept \
    { \
        return T(lhs.value / rhs); \
    } \
    friend constexpr ScalarType operator/(T lhs, T rhs) noexcept \
    { \
        return lhs.value / rhs.value; \
    } \
    friend constexpr T operator%(T lhs, T rhs) noexcept \
    { \
        return T(std::fmod(lhs.value, rhs.value)); \
    } \
 \
    constexpr T& operator+=(T other) noexcept \
    { \
        this->value += other.value; \
        return *this; \
    } \
    constexpr T& operator-=(T other) noexcept \
    { \
        this->value -= other.value; \
        return *this; \
    } \
    constexpr T& operator*=(ScalarType other) noexcept \
    { \
        this->value *= other; \
        return *this; \
    } \
    constexpr T& operator/=(ScalarType other) noexcept \
    { \
        this->value /= other; \
        return *this; \
    } \
    constexpr T& operator%=(T other) noexcept \
    { \
        this->value = std::fmod(this->value, other.value); \
        return *this; \
    }

    #pragma pack(push, 1)

    struct UnsignedFixedDuration
    {
        __implement_unit_universal(UnsignedFixedDuration, uint32_t)

        constexpr uint32_t milliSeconds() const noexcept
        {
            return this->value;
        }
    private:
        uint32_t value;
    };
    struct Duration
    {
        __implement_unit_universal(Duration, float)

        constexpr operator UnsignedFixedDuration() const noexcept
        {
            return UnsignedFixedDuration(boundedCast<uint32_t>(this->value * 1000.0f));
        }

        constexpr float seconds() const noexcept
        {
            return this->value;
        }
    private:
        float value;
    };

    #pragma pack(pop)
}

#define _msu atmc::UnsignedFixedDuration(1);
#define _su atmc::UnsignedFixedDuration(1000);

#define _ns atmc::Duration(0.000000001f)
#define _us atmc::Duration(0.000001f)
#define _ms atmc::Duration(0.001f)
#define _s atmc::Duration(1.0f)
#define _min atmc::Duration(60.0f)
#define _h atmc::Duration(3600.0f)
