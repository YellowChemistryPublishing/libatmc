/* module; */
#pragma once

#include <cassert>
#include <coroutine>
#include <cstdint>
#include <cstdio>
#include <cxxutil.h>
#include <entry.h>
#include <list>
#include <runtime_headers.h>

#include <FreeRTOS.h>
#include <task.h>

/* export module core.Concurrency; */

#include <Config.h>
/* import core.Containers; */
#include <InplaceQueue.hpp>

/* export */ namespace atmc
{
    template <typename T>
    struct TaskPromise;

    template <typename T>
    class Task;

    template <typename T>
    struct TaskAwaiter
    {
        std::coroutine_handle<TaskPromise<T>> handle;

        inline bool await_ready() const noexcept
        {
            return !this->handle || this->handle.done();
        }
        inline std::coroutine_handle<> await_suspend(std::coroutine_handle<> parent)
        {
            this->handle.promise().continuation = parent;
            taskYIELD();
            return this->handle;
        }
        inline T await_resume() const noexcept(std::is_same<T, void>::value)
        {
            if constexpr (!std::is_same<T, void>::value)
                return std::move(*reinterpret_cast<T*>(this->handle.promise().value));
        }
    };
    template <typename T>
    struct TaskFinalAwaiter
    {
        std::coroutine_handle<TaskPromise<T>> handle;

        inline bool await_ready() const noexcept
        {
            return false;
        }
        template <typename PromiseType>
        inline std::coroutine_handle<> await_suspend(std::coroutine_handle<PromiseType> handle) noexcept
        {
            taskYIELD();
            return this->handle.promise().continuation;
        }
        inline void await_resume() const noexcept
        { }
    };

    template <typename T>
    struct TaskPromiseCore
    {
        std::coroutine_handle<> continuation;

        inline std::suspend_always initial_suspend() noexcept
        {
            return std::suspend_always();
        }
        
        [[noreturn]] inline static Task<T> get_return_object_on_allocation_failure()
        {
            __throw(std::bad_alloc());
        }
        inline void unhandled_exception()
        {
            std::rethrow_exception(std::current_exception());
        }
    private:
        static uint_least8_t memAvail[Config::TaskCoroutineBlockSize];
        static size_t memAvailEnd;
    };
    template <typename T>
    struct TaskPromise final : public TaskPromiseCore<T>
    {
        alignas(T) unsigned char value[sizeof(T)];

        inline TaskFinalAwaiter<T> final_suspend() noexcept
        {
            return TaskFinalAwaiter<T> { std::coroutine_handle<TaskPromise<T>>::from_promise(*this) };
        }

        inline Task<T> get_return_object();

        template <typename ReturnType>
        inline void return_value(ReturnType&& ret)
        {
            new(this->value) T(std::forward<ReturnType>(ret));
        }
    };
    template <>
    struct TaskPromise<void> final : public TaskPromiseCore<void>
    {
        inline TaskFinalAwaiter<void> final_suspend() noexcept
        {
            return TaskFinalAwaiter<void> { std::coroutine_handle<TaskPromise<void>>::from_promise(*this) };
        }

        inline Task<void> get_return_object();

        inline void return_void()
        { }
    };

    template <typename T = void>
    class [[nodiscard]] Task final
    {
    public:
        using promise_type = TaskPromise<T>;

        constexpr static uint32_t MaxDelay = HAL_MAX_DELAY;

        inline ~Task()
        {
            if (this->handle)
                this->handle.destroy();
        }

        inline TaskAwaiter<T> operator co_await()
        {
            return TaskAwaiter<T>(this->handle);
        }

        inline static Task<void> yield() requires (std::is_same<T, void>::value)
        {
            co_return;
        }
        static Task<void> delay(uint32_t ms) requires (std::is_same<T, void>::value)
        {
            uint32_t from = HAL_GetTick();
            Task<void> ret = [](uint32_t from, uint32_t ms) -> Task<void>
            {
                while (HAL_GetTick() - from < ms)
                    co_await Task<>::yield();
            }(from, ms);
            return ret;
        }

        friend struct atmc::TaskPromise<T>;
    private:
        std::coroutine_handle<TaskPromise<T>> handle;

        inline explicit Task(std::coroutine_handle<TaskPromise<T>> handle) : handle(handle)
        { }
    };

    template <typename T>
    Task<T> TaskPromise<T>::get_return_object()
    {
        return Task<T>(std::coroutine_handle<TaskPromise<T>>::from_promise(*this));
    }
    Task<void> TaskPromise<void>::get_return_object()
    {
        return Task<void>(std::coroutine_handle<TaskPromise<void>>::from_promise(*this));
    }

    template <typename T, bool IsAsync>
    using TaskIfAsync = std::conditional<IsAsync, Task<T>, T>::type;

    struct Async;

    struct AsyncPromise
    {
        inline Async get_return_object();

        inline std::suspend_always initial_suspend() noexcept
        {
            return std::suspend_always();
        }
        inline std::suspend_never final_suspend() noexcept
        {
            return std::suspend_never();
        }

        inline void unhandled_exception() noexcept
        {
            std::rethrow_exception(std::current_exception());
        }
        inline void return_void() noexcept
        { }
    };

    struct Async
    {
        using promise_type = AsyncPromise;

        friend struct AsyncPromise;
    private:
        inline explicit Async(std::coroutine_handle<AsyncPromise> handle)
        {
            xTaskCreate([](void* pvParams)
            {
                auto handle = std::coroutine_handle<AsyncPromise>::from_address(pvParams);
                handle.resume();
                vTaskDelete(nullptr);
            }, "Async", 512, handle.address(), 1, nullptr);
        }
    };

    Async AsyncPromise::get_return_object()
    {
        return Async(std::coroutine_handle<AsyncPromise>::from_promise(*this));
    }
}
