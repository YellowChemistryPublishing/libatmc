#pragma once

#include <cxxutil.hpp>

#include <FreeRTOS.h>
#include <task.h>

namespace atmc
{
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