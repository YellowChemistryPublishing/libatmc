#pragma once

#include <cstddef>

namespace atmc
{
    struct Config final
    {
        Config() = delete;

        // v General
                
        constexpr static int PinCountGPIO = 16;
        constexpr static int I2CBusCount = 4;
        constexpr static int SPIBusCount = 3;

        constexpr static size_t TaskCoroutineBlockSize = 16384;

        // ^ General
    };
}