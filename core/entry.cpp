#include <entry.h>
#include <exception>
#include <print>

#include <Exception.h>
#include <GPIOManager.h>
#include <LanguageSupport.h>
#include <Task.h>

using namespace atmc;
using namespace sys;

void* operator new(size_t sz)
{
    void* ret = pvPortMalloc(sz);
    if (!ret)
        _throw(std::bad_alloc());
    else
        return ret;
}
void operator delete(void* p) noexcept
{
    vPortFree(p);
}
void operator delete(void* p, size_t) noexcept
{
    vPortFree(p);
}

extern "C" __weak void init()
{ }

// `try`-`catch` blocks are rarely used in embedded systems, and I've never gotten them to work properly.
// If they are, they should be used to catch exceptions thrown by the `init` and `tick` functions.
// Otherwise, sufficient error messages are printed to `stderr` anyways.

void __initHandler()
{
    _push_nowarn_gcc(_clWarn_gcc_use_after_free);
    _push_nowarn_clang(_clWarn_clang_use_after_free);
    try
    {
        init();

        vTaskStartScheduler();
    }
    catch (const std::exception& ex)
    {
        std::println(stderr, "Exception thrown during call to `extern \"C\" void init()`: {}", ex.what());
        _throw(terminate_exception());
    }
    catch (...)
    {
        std::println(stderr, "Unmanaged exception of type `{}` thrown during call to `extern \"C\" void init()`.", exception_type_name(std::current_exception()).get());
        _throw(terminate_exception());
    }
    _pop_nowarn_clang();
    _pop_nowarn_gcc();
}
