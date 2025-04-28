#include <entry.h>

#include <Config.h>
#include <GPIOManager.hpp>
#include <Result.hpp>
#include <Task.hpp>
#include <Thread.hpp>

using namespace atmc;

struct ExampleConfig final
{
    ExampleConfig() = delete;
    
    constexpr static GPIOPin BlinkyPin = GPIOPin::PC6;
    constexpr static int BlinkyAlternateDuration = 1000;
};

Async entryPoint()
{
    AnalogPin initPins[] = { AnalogPin(0, 16), AnalogPin(0, 18) };
    __fence_contract_enforce(GPIOManager::initADC(initPins) == HardwareStatus::Ok);

    while (true)
    {
        auto a = (co_await GPIOManager::analogRead(AnalogPin(0, 16))).valueOrThrow();
        auto b = (co_await GPIOManager::analogRead(AnalogPin(0, 18))).valueOrThrow();
        std::println("{}\t{}", a, b);
        co_await Task<>::delay(400);
    }
}

void init()
{
    entryPoint();
}
