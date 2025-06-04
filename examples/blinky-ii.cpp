#include <print>

#include <module/core.IO.Embedded>
#include <module/core>
#include <module/sys>

using namespace sys;
using namespace atmc;

struct ExampleConfig final
{
    ExampleConfig() = delete;

    constexpr static AnalogPin potPin = AnalogPin(0, 0);
    constexpr static PWMPin ledPin = PWMPin(0, 0);
};

async entryPoint()
{
    while (true)
    {
        float dc = (co_await GPIOManager::analogRead(ExampleConfig::potPin)).expect();
        std::println("LED Brightness: {}.", dc);

        if (dc < 0.04f)
            dc = 0.0f;

        GPIOManager::pwmWrite(ExampleConfig::ledPin, dc);

        co_await task<>::delay(50);
    }
}

void init()
{
    entryPoint();
}
