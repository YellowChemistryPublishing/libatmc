#pragma once

#include <cstdint>
#include <runtime_headers.h>

#include <FreeRTOS.h>
#include <task.h>

namespace atmc
{
    class Timer
    {
        TickType_t beg;
    public:
        constexpr Timer() = default;

        inline void start()
        {
            this->beg = xTaskGetTickCount();
        }
        inline TickType_t elapsed() const
        {
            return pdTICKS_TO_MS(xTaskGetTickCount() - this->beg);
        }
    };
}