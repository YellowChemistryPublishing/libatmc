#include <cstdio>
#include <entry.h>
#include <exception>
#include <module/sys>
#include <print>
#include <rt_threading.h>

#include <LanguageSupport.h>
#include <Target.h>

#if _libatmc_target_stm32
#include <Task.h>
#endif

using namespace atmc;

#if _libatmc_target_stm32
void* operator new(size_t sz)
{
    void* ret = pvPortMalloc(sz);
    if (!ret)
        _throw(std::bad_alloc());
    else
        return ret;
}
void operator delete(void* p) noexcept { vPortFree(p); }
void operator delete(void* p, size_t) noexcept { vPortFree(p); }
#endif

extern "C" _weak void init() { }

// `try`-`catch` blocks are rarely used in embedded systems, and I've never gotten them to work properly.
// If they are, they should be used to catch exceptions thrown by the `init` and `tick` functions.
// Otherwise, sufficient error messages are printed to `stderr` anyways.

void __initHandler()
{
    _push_nowarn_gcc(_clwarn_gcc_use_after_free);
    _push_nowarn_clang(_clwarn_clang_use_after_free);
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
        _throw(sys::terminate_exception());
    }
    catch (...)
    {
        std::println(stderr, "Unmanaged exception of type `{}` thrown during call to `extern \"C\" void init()`.", sys::exception_type_name(std::current_exception()).get());
        _throw(sys::terminate_exception());
    }
    _pop_nowarn_clang();
    _pop_nowarn_gcc();
}

// NOLINTEND(misc-include-cleaner)
