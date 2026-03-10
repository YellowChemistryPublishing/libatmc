#include <cstdio>
#include <entry.h>
#include <exception>
#include <new> // NOLINT(misc-include-cleaner)
#include <print>
#include <rt_threading.h>

#include <module/sys>

#include <Target.h>

// NOLINTBEGIN(misc-include-cleaner)

#if _libatmc_target_stm32

#include <FreeRTOS.h>

#include <Task.h>

#endif

using namespace atmc;

#if _libatmc_target_stm32
void* operator new(size_t sz) // NOLINT(readability-inconsistent-declaration-parameter-name)
{
    void* const ret = pvPortMalloc(sz);
    if (!ret)
        throw std::bad_alloc();
    return ret;
}
void operator delete(void* p) noexcept { vPortFree(p); }         // NOLINT(readability-inconsistent-declaration-parameter-name)
void operator delete(void* p, size_t) noexcept { vPortFree(p); } // NOLINT(readability-inconsistent-declaration-parameter-name)
#endif

extern "C" _weak void init() { }

// `try`-`catch` blocks are rarely used in embedded systems, and I've never gotten them to work properly.
// If they are, they should be used to catch exceptions thrown by the `init` and `tick` functions.
// Otherwise, sufficient error messages are printed to `stderr` anyways.

void __initHandler()
{
    try
    {
#if _libatmc_target_hosted
        const sys::platform::thread_pool pool;
#endif

        init();

#if _libatmc_target_stm32
        vTaskStartScheduler();
#endif
    }
    catch (const std::exception& ex)
    {
        std::println(stderr, "Exception thrown during call to `extern \"C\" void init()`: {}", ex.what());
        _contract_assert(false, "Exception thrown in `extern \"C\" void init()`.");
    }
    catch (...)
    {
        std::println(stderr, "Unknown exception thrown during call to `extern \"C\" void init()`.");
        _contract_assert(false, "Exception thrown in `extern \"C\" void init()`.");
    }
}

// NOLINTEND(misc-include-cleaner)
