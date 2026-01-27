#include <module/sys.Threading>

struct ExampleConfig final
{
    ExampleConfig() = delete;
};

sys::async entryPoint() { co_return; }

void init() { entryPoint(); }
