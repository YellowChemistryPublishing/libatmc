#pragma once

#include <limits>

#include <FreeRTOS.h>
#include <stm32h7xx_hal_def.h>
#include <task.h>

#include <LanguageSupport.h>

namespace sys::platform
{
    struct ThreadCriticalSectionISR
    {
        _inline_always ThreadCriticalSectionISR() : irqStatus(taskENTER_CRITICAL_FROM_ISR())
        { }
        _inline_always ~ThreadCriticalSectionISR()
        {
            taskEXIT_CRITICAL_FROM_ISR(this->irqStatus);
        }
    private:
        UBaseType_t irqStatus;
    };
    struct ThreadCriticalSection
    {
        _inline_always ThreadCriticalSection()
        {
            taskENTER_CRITICAL();
        }
        _inline_always ~ThreadCriticalSection()
        {
            taskEXIT_CRITICAL();
        }
    };

    using ThreadID = UBaseType_t;

    class ThreadHandle
    {
        TaskHandle_t handle;

        inline ThreadHandle(TaskHandle_t handle) : handle(handle)
        { }
    public:
        inline static ThreadHandle currentThread()
        {
            ThreadCriticalSection guard;
            return xTaskGetCurrentTaskHandle();
        }

        inline ThreadID id()
        {
            return uxTaskGetTaskNumber(this->handle);
        }
    };

    constexpr i32 _task_max_delay(std::numeric_limits<i32::underlying_type>::max());
} // namespace sys::platform

#define _impl_task_yield()                                                            \
    inline static auto yield()                                                        \
    requires (std::is_same<T, void>::value)                                           \
    {                                                                                 \
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
        return ret;                                                                   \
    }
#define _impl_task_delay()                                                               \
    inline static task<void> delay(u32 ms)                                               \
    requires (std::is_same<T, void>::value)                                              \
    {                                                                                    \
        u32 from = xTaskGetTickCount();                                                  \
        while (pdTICKS_TO_MS(xTaskGetTickCount() - from) < ms) co_await task<>::yield(); \
    }
#define _task_yield_and_resume() \
    taskYIELD();                 \
    return this->handle
#define _task_yield_and_continue()             \
    taskYIELD();                               \
    return this->handle.promise().continuation
