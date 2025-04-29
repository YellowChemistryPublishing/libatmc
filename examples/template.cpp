#include <Task.hpp>

using namespace atmc;

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

Async entryPoint()
{
    co_return;
}

void init()
{
    entryPoint();
}
