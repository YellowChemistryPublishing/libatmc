#pragma once

#include <limits>

#include <FreeRTOS.h>
#include <stm32h7xx_hal_def.h>
#include <task.h>

#include <LanguageSupport.h>

namespace sys::platform
{
    struct thread_critical_section_isr
    {
        _inline_always thread_critical_section_isr() : irqStatus(taskENTER_CRITICAL_FROM_ISR()) { }
        _inline_always ~thread_critical_section_isr() { taskEXIT_CRITICAL_FROM_ISR(this->irqStatus); }
    private:
        UBaseType_t irqStatus;
    };
    struct thread_critical_section
    {
        _inline_always thread_critical_section() { taskENTER_CRITICAL(); }
        _inline_always ~thread_critical_section() { taskEXIT_CRITICAL(); }
    };

    using thread_id = UBaseType_t;

    class thread_handle
    {
        TaskHandle_t handle;

        inline thread_handle(TaskHandle_t handle) : handle(handle) { }
    public:
        inline static thread_handle currentThread()
        {
            thread_critical_section guard;
            return xTaskGetCurrentTaskHandle();
        }

        inline thread_id id() { return uxTaskGetTaskNumber(this->handle); }
    };

    constexpr i32 task_max_delay = i32::highest();
} // namespace sys::platform

#define _impl_task_yield_rtype auto
#define _impl_task_yield()                                                            \
    struct                                                                            \
    {                                                                                 \
        _inline_always constexpr bool await_ready() const noexcept                    \
        {                                                                             \
            taskYIELD();                                                              \
            return true;                                                              \
        }                                                                             \
        _inline_always void await_suspend(std::coroutine_handle<>) const noexcept { } \
        _inline_always constexpr void await_resume() const noexcept { }               \
    } ret;                                                                            \
    return ret;

#define _impl_task_delay_rtype ::sys::task<void>
#define _impl_task_delay()                                      \
    TickType_t from = xTaskGetTickCount();                      \
    while (i32(pdTICKS_TO_MS(xTaskGetTickCount() - from)) < ms) \
        co_await task<>::yield();

#define _task_yield_and_resume() \
    taskYIELD();                 \
    return this->handle

#define _task_yield_and_continue()             \
    taskYIELD();                               \
    return this->handle.promise().continuation
