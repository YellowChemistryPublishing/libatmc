#pragma once

#include <cstddef>
#include <initializer_list>

namespace atmc
{
    struct Config final
    {
        Config() = delete;

        // v General

        static constexpr int AsyncThreadPriority = 1;
        static constexpr int AsyncThreadStackSizeWords = 2048;

        static constexpr int TaskPromiseStackSize = 4096;

        // Probably shouldn't change... v / ^ General

        static constexpr int PinCountGPIO = 16;

        static constexpr int MaxADCChannels = 20;
        static constexpr int AnalogConverterCount = 3;

        static constexpr int I2CBusCount = 4;
        static constexpr int SPIBusCount = 3;

        // ^ Probably shouldn't change...
    };
} // namespace atmc
