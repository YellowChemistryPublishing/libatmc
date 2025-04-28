#pragma once

#include <cstddef>
#include <initializer_list>

namespace atmc
{
    struct Config final
    {
        Config() = delete;

        // v General
        
        constexpr static int PinCountGPIO = 16;

        constexpr static int MaxADCChannels = 20;
        constexpr static int AnalogConverterCount = 3;

        constexpr static int I2CBusCount = 4;
        constexpr static int SPIBusCount = 3;

        constexpr static int AsyncThreadPriority = 1;
        constexpr static int AsyncThreadStackSizeWords = 2048;

        // ^ General
    };
}