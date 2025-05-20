#pragma once

#include <FreeRTOS.h>
#include <task.h>

#include <LanguageSupport.h>

namespace sys
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

    class Thread
    {
        TaskHandle_t handle;

        inline Thread(TaskHandle_t handle) : handle(handle)
        { }
    public:
        inline static Thread currentThread()
        {
            ThreadCriticalSection guard;
            return Thread(xTaskGetCurrentTaskHandle());
        }

        inline ThreadID id()
        {
            return uxTaskGetTaskNumber(this->handle);
        }
    };
} // namespace sys