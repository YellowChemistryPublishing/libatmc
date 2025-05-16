#include <entry.h>

#include <module/core.IO.Embedded.hpp>
#include <module/sys.hpp>
#include <module/sys.Threading.hpp>

using namespace atmc;
using namespace sys;

struct ExampleConfig final
{
    ExampleConfig() = delete;
    
    constexpr static GPIOPin BlinkyPin = GPIOPin::PC6;
    constexpr static int BlinkyAlternateDuration = 1000;
};

Async entryPoint()
{
    GPIOPin::State pinState = GPIOPin::High;
    while (true)
    {
        GPIOManager::digitalWrite(ExampleConfig::BlinkyPin, pinState);
        std::println("Blinking {}!", pinState == GPIOPin::High ? "high" : "low");
        if (pinState == GPIOPin::High)
            pinState = GPIOPin::Low;
        else pinState = GPIOPin::High;
        
        co_await Task<>::delay(ExampleConfig::BlinkyAlternateDuration);
    }
}

void init()
{
    entryPoint();
}
