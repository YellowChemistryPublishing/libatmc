#include <algorithm>
#include <utility>

#include <GPIOManager.h>
#include <TaskEx.h>

using namespace atmc;
using namespace sys;

// For an Interlink Electronics FSR 406.
struct ExampleConfig final
{
    ExampleConfig() = delete;

    constexpr static AnalogPin fsrPin = AnalogPin(0, 0);

    constexpr static float seriesAddedResistance = 1000.0f;
    //                         kOhm   g
    constexpr static std::pair<float, float> gramForceLookupTable[] { { 100.0f, 0.0f },
                                                                      { 30.0f, 20.0f },
                                                                      { 10.0f, 50.0f },
                                                                      { 6.0f, 100.0f },
                                                                      { 3.5f, 250.0f },
                                                                      { 2.0f, 500.0f },
                                                                      { 1.25f, 1000.0f },
                                                                      { 0.75f, 2000.0f },
                                                                      { 0.45f, 4000.0f },
                                                                      { 0.3f, 7000.0f },
                                                                      { 0.15f, 10000.0f },
                                                                      { 0.0f, 100000.0f } };
};

float lerpTable(float resistance)
{
    const sz tableSize = std::size(ExampleConfig::gramForceLookupTable);
    sz i = 0_uz;

    if (resistance <= ExampleConfig::gramForceLookupTable[+(tableSize - 1_uz)].first)
        return ExampleConfig::gramForceLookupTable[+(tableSize - 1_uz)].second;

    while (i < tableSize && resistance < ExampleConfig::gramForceLookupTable[+i].first) ++i;

    const std::pair<float, float>& left = ExampleConfig::gramForceLookupTable[+(i - 1_uz)];
    const std::pair<float, float>& right = ExampleConfig::gramForceLookupTable[+i];
    return std::max(0.0f, std::lerp(left.second, right.second, (left.first - resistance) / (left.first - right.first)));
}

async entryPoint()
{
    while (true)
    {
        // vDD = vRead + vFSR = i 1kOhm + i Rfsr
        // => 1k / (1k + Rfsr) = vRead / vDD
        // => Rfsr = 1k / (vRead / vDD) - 1k

        //                    vRead
        //                      o
        //                      |
        // GND o---[ 1 kOhm ]---+---[ vres /^ ]---o vDD

        float resistance = (1000.0f / (co_await GPIOManager::analogRead(ExampleConfig::fsrPin)).expect() - 1000.0f) / 1000.0f;
        std::println("{} g ({} kOhm)", lerpTable(resistance), resistance);
        co_await sys::task<>::delay(20);
    }

    co_return;
}

void init()
{
    entryPoint();
}
