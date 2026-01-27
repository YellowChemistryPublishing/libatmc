#include <print>

#include <module/core.IO.Embedded>
#include <module/sys.Threading>
#include <module/sys>

using namespace atmc;

struct ExampleConfig final
{
    ExampleConfig() = delete;

#if _libatmc_target_stm32
    static constexpr GPIOPin BlinkyPin = GPIOPin::PC6;
#else
    static constexpr GPIOPin BlinkyPin = GPIOPin::PZ9;
#endif
    static constexpr int BlinkyAlternateDuration = 1000;
};

sys::async entryPoint()
{
    GPIOPin::State pinState = GPIOPin::High;
    while (true)
    {
        GPIOManager::digitalWrite(ExampleConfig::BlinkyPin, pinState);
        std::println("Blinking {}!", pinState == GPIOPin::High ? "high" : "low");
        if (pinState == GPIOPin::High)
            pinState = GPIOPin::Low;
        else
            pinState = GPIOPin::High;

        co_await task<>::delay(i32(ExampleConfig::BlinkyAlternateDuration));
    }
}

void init() { entryPoint(); }
