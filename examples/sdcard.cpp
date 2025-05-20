#include <cxxsup.h>
#include <initializer_list>
#include <vector>

#include <StringEx.h>
#include <TaskEx.h>

using namespace sys;

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
