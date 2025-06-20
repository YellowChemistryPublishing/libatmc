#include <GPIOManager.h>
#include <TaskEx.h>

using namespace atmc;
using namespace sys;

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

async entryPoint()
{
    while (true)
    {
        std::println("{}", (co_await GPIOManager::analogRead(AnalogPin(0, 0))).expect());
        co_await sys::task<>::delay(20);
    }

    co_return;
}

void init()
{
    entryPoint();
}
