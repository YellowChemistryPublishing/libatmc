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
    
    constexpr static AnalogPin potPin = AnalogPin(0, 0);
    constexpr static int BlinkyAlternateDuration = 1000;
};

Async entryPoint()
{
    while (true)
    {
        float dc = (co_await GPIOManager::analogRead(ExampleConfig::potPin)).valueOrThrow();
        std::println("LED Brightness: {}.", dc);
        co_await Task<>::delay(50);
    }
}

void init()
{
    entryPoint();
}
