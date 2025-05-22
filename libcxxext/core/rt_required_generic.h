#pragma once

#include <thread>

struct __tcs_isr
{
    _inline_always __tcs_isr()
    { }
    _inline_always ~__tcs_isr()
    { }
};
struct __tcs
{
    _inline_always __tcs()
    { }
    _inline_always ~__tcs()
    { }
};

using __thread_id = std::thread::id;

class __thread_type
{
    void* handle;

    inline __thread_type(void* handle) : handle(handle)
    { }
public:
    [[noreturn]] inline static __thread_type currentThread()
    { }

    [[noreturn]] inline __thread_id id()
    { }
};

#define __task_yield_to_sched() co_return;
#define __task_yield_and_resume()
#define __task_yield_and_continue()
