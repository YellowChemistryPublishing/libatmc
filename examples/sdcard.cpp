#include <cxxsup.h>
#include <print>

#include <ManagedArray.h>
#include <StringEx.h>
#include <TaskEx.h>

using namespace sys;

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

async entryPoint()
{
    auto arr = managed_array<int>::ctor(23);
    auto arrdone = arr.move();
    std::println("{}", arrdone[0].move());

    co_return;
}

void init()
{
    entryPoint();
}
