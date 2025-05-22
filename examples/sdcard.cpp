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

Async entryPoint()
{
    auto arr = ManagedArray<int>::ctor(23);
    auto arrdone = arr.move();
    std::println("{}", arr[0].move());

    co_return;
}

void init()
{
    entryPoint();
}
