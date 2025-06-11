#include <TaskEx.h>

using namespace sys;

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

async entryPoint()
{
    co_return;
}

void init()
{
    entryPoint();
}
