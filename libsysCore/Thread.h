#pragma once

#include <cxxutil.h>

#include <FreeRTOS.h>
#include <task.h>

namespace sys
{
    struct ThreadCriticalSectionISR
    {
        __inline_always ThreadCriticalSectionISR() : irqStatus(taskENTER_CRITICAL_FROM_ISR())
        { }
        __inline_always ~ThreadCriticalSectionISR()
        {
            taskEXIT_CRITICAL_FROM_ISR(this->irqStatus);
        }
    private:
        UBaseType_t irqStatus;
    };
    struct ThreadCriticalSection
    {
        __inline_always ThreadCriticalSection()
        {
            taskENTER_CRITICAL();
        }
        __inline_always ~ThreadCriticalSection()
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
}