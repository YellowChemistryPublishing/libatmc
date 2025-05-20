#include <cxxsup.h>
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
    std::vector<string> ss;
    [[maybe_unused]] String s = string::concat(ss);

    co_return;
}

void init()
{
    entryPoint();
}
