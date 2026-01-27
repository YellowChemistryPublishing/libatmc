#include <print>

#include <module/core.IO.Embedded>
#include <module/core>
#include <module/sys>

using namespace atmc;

struct ExampleConfig final
{
    ExampleConfig() = delete;

    static constexpr AnalogPin potPin = AnalogPin(0, 0);
    static constexpr PWMPin ledPin = PWMPin(0, 0);
};

sys::async entryPoint()
{
    while (true)
    {
        float dc = (co_await GPIOManager::analogRead(ExampleConfig::potPin)).expect();
        std::println("LED Brightness: {}.", dc);

        if (dc < 0.04f)
            dc = 0.0f;

        GPIOManager::pwmWrite(ExampleConfig::ledPin, dc);

        co_await sys::task<>::delay(50_i32);
    }
}

void init() { entryPoint(); }
