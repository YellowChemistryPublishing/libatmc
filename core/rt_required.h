#pragma once

#include <FreeRTOS.h>
#include <task.h>

#include <LanguageSupport.h>

struct __tcs_isr
{
    _inline_always __tcs_isr() : irqStatus(taskENTER_CRITICAL_FROM_ISR())
    { }
    _inline_always ~__tcs_isr()
    {
        taskEXIT_CRITICAL_FROM_ISR(this->irqStatus);
    }
private:
    UBaseType_t irqStatus;
};
struct __tcs
{
    _inline_always __tcs()
    {
        taskENTER_CRITICAL();
    }
    _inline_always ~__tcs()
    {
        taskEXIT_CRITICAL();
    }
};

using __thread_id = UBaseType_t;

class __thread_type
{
    TaskHandle_t handle;

    inline __thread_type(TaskHandle_t handle) : handle(handle)
    { }
public:
    inline static __thread_type currentThread()
    {
        __tcs guard;
        return __thread_type(xTaskGetCurrentTaskHandle());
    }

    inline __thread_id id()
    {
        return uxTaskGetTaskNumber(this->handle);
    }
};

#define __task_yield_to_sched()                                                   \
    struct                                                                        \
    {                                                                             \
        _inline_always constexpr bool await_ready() const noexcept                \
        {                                                                         \
            taskYIELD();                                                          \
            return true;                                                          \
        }                                                                         \
        _inline_always void await_suspend(std::coroutine_handle<>) const noexcept \
        { }                                                                       \
        _inline_always constexpr void await_resume() const noexcept               \
        { }                                                                       \
    } ret;                                                                        \
    return ret
#define __task_yield_and_resume() \
    taskYIELD();                  \
    return this->handle
#define __task_yield_and_continue()             \
    taskYIELD();                                \
    return this->handle.promise().continuation;
