#pragma once

#include <cstdint>
#include <runtime_headers.h>

namespace atmc
{
    class Timer
    {
        uint32_t beg;
    public:
        constexpr Timer() = default;

        inline void start()
        {
            this->beg = HAL_GetTick();
        }
        inline uint32_t elapsed() const
        {
            return HAL_GetTick() - this->beg;
        }
    };
}