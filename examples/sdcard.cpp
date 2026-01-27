#include <print>

#include <TaskEx.h>

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

async entryPoint() { co_return; }

void init() { entryPoint(); }
