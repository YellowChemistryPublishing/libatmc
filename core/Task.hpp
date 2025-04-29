/* module; */
#pragma once

#include <cassert>
#include <coroutine>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
    /// @brief
    /// @note Static class.
    class TaskAllocator
    {
        struct ChunkHeader
        {
            size_t prevSize;
            size_t size;
            bool isFree;
        };

        alignas(std::max_align_t) static unsigned char stack[2048];
        static size_t stackSize;
    public:
        TaskAllocator() = delete;
        
        inline static void* stackNew(size_t sz) noexcept
        {
            void* alignedPtr = &stack[stackSize] + sizeof(bool) + sizeof(size_t) * 2;
            if (stackSize + sizeof(bool) + sizeof(size_t) * 2 + sz > 2048)
                return nullptr;
            [[gnu::unused]] size_t alignedSize = 2048 - (stackSize + sizeof(bool) + sizeof(size_t) * 2);
            std::align(alignof(std::max_align_t), sz, alignedPtr, alignedSize);
            unsigned char* ret = __sc(unsigned char*, alignedPtr);
            if (ret + sz > stack + 2048)
                return nullptr;
            
            memcpy(ret - sizeof(size_t) * 2, &stackSize, sizeof(size_t));

            {
                bool _false = false;
                memcpy(ret - sizeof(size_t) * 2 - sizeof(bool), &_false, sizeof(bool));
            }

            stackSize = ret + sz - stack;

            return ret;
        }
        inline static void stackDelete(void* ptr) noexcept
        {
            unsigned char* ret = __sc(unsigned char*, ptr);
            size_t sz = *(size_t*)(ret - sizeof(size_t));
            size_t& stackSize = *(size_t*)(ret - sizeof(size_t) - sizeof(bool));
            stackSize = ret - stack;
        }
    };

    template <typename T>
    struct TaskPromise;

    template <typename T>
    class Task;

    template <typename T>
    struct TaskAwaiter
    {
        std::coroutine_handle<TaskPromise<T>> handle;

        __inline_always constexpr bool await_ready() const noexcept
        {
            return !this->handle || this->handle.done();
        }
        __inline_never std::coroutine_handle<> await_suspend(std::coroutine_handle<> parent)
        {
            this->handle.promise().continuation = parent;
            taskYIELD();
            return this->handle;
        }
        __inline_always constexpr T await_resume() const noexcept(std::is_same<T, void>::value)
        {
            if (this->handle.promise().exception) [[unlikely]]
                std::rethrow_exception(this->handle.promise().exception);
            if constexpr (!std::is_same<T, void>::value)
                return std::move(*reinterpret_cast<T*>(this->handle.promise().value));
        }
    };
    template <typename T>
    struct TaskFinalAwaiter
    {
        std::coroutine_handle<TaskPromise<T>> handle;

        __inline_always constexpr bool await_ready() const noexcept
        {
            return false;
        }
        __inline_never std::coroutine_handle<> await_suspend(std::coroutine_handle<> handle) noexcept
        {
            taskYIELD();
            return this->handle.promise().continuation;
        }
        constexpr void await_resume() const noexcept
        { }
    };

    template <typename T>
    struct TaskPromiseCore
    {
        std::coroutine_handle<> continuation;
        std::exception_ptr exception = nullptr;

        __inline_always constexpr std::suspend_always initial_suspend() const noexcept
        {
            return std::suspend_always();
        }
        
        __inline_always Task<T> get_return_object()
        {
            return Task<T>(std::coroutine_handle<TaskPromise<T>>::from_promise(*static_cast<TaskPromise<T>*>(this)));
        }

        [[noreturn]] inline static Task<T> get_return_object_on_allocation_failure()
        {
            __throw(std::bad_alloc());
        }
        __inline_always void unhandled_exception()
        {
            this->exception = std::current_exception();
        }
    };
    template <typename T>
    struct TaskPromise final : public TaskPromiseCore<T>
    {
        alignas(T) unsigned char value[sizeof(T)];

        __inline_always TaskFinalAwaiter<T> final_suspend() noexcept
        {
            return TaskFinalAwaiter<T> { std::coroutine_handle<TaskPromise<T>>::from_promise(*this) };
        }

        template <typename ReturnType>
        __inline_always constexpr void return_value(ReturnType&& ret)
        {
            new(this->value) T(std::forward<ReturnType>(ret));
        }
    };
    template <>
    struct TaskPromise<void> final : public TaskPromiseCore<void>
    {
        __inline_always TaskFinalAwaiter<void> final_suspend() noexcept
        {
            return TaskFinalAwaiter<void> { std::coroutine_handle<TaskPromise<void>>::from_promise(*this) };
        }

        __inline_always constexpr void return_void() const noexcept
        { }
    };

    template <typename T = void>
    class [[nodiscard]] Task final
    {
    public:
        using promise_type = TaskPromise<T>;

        constexpr static uint32_t MaxDelay = HAL_MAX_DELAY;

        __inline_always ~Task()
        {
            if (this->handle)
                this->handle.destroy();
        }

        __inline_always TaskAwaiter<T> operator co_await()
        {
            return TaskAwaiter<T>(this->handle);
        }

        inline static auto yield() requires (std::is_same<T, void>::value)
        {
            struct
            {
                __inline_always constexpr bool await_ready() const noexcept
                {
                    taskYIELD();
                    return true;
                }
                __inline_always void await_suspend(std::coroutine_handle<> parent) const noexcept
                { }
                __inline_always constexpr void await_resume() const noexcept
                { }
            } ret;
            return ret;
        }
        inline static Task<void> delay(uint32_t ms) requires (std::is_same<T, void>::value)
        {
            uint32_t from = xTaskGetTickCount();
            while (pdTICKS_TO_MS(xTaskGetTickCount() - from) < ms)
                co_await Task<>::yield();
        }
        template <typename Pred>
        inline static Task<void> waitUntil(Pred&& func)
        {
            if constexpr (!std::convertible_to<decltype(func()), bool>)
                while (!co_await func());
            else while (!func())
                co_await Task<>::yield();
        }

        friend struct atmc::TaskPromiseCore<T>;
    private:
        std::coroutine_handle<TaskPromise<T>> handle;

        __inline_always explicit Task(std::coroutine_handle<TaskPromise<T>> handle) : handle(handle)
        {
            handle.resume();
        }
    };

    template <typename T, bool IsAsync>
    using TaskIfAsync = std::conditional<IsAsync, Task<T>, T>::type;

    struct Async;

    struct AsyncPromise
    {
        consteval AsyncPromise() noexcept = default;
        __inline_always constexpr ~AsyncPromise() = default;

        __inline_always Async get_return_object();

        __inline_always constexpr std::suspend_always initial_suspend() const noexcept
        {
            return std::suspend_always();
        }
        __inline_always constexpr std::suspend_never final_suspend() const noexcept
        {
            return std::suspend_never();
        }

        inline void unhandled_exception() noexcept
        {
            std::rethrow_exception(std::current_exception());
        }
        __inline_always constexpr void return_void() const noexcept
        { }
    };

    struct Async
    {
        using promise_type = AsyncPromise;

        __inline_always constexpr ~Async() = default;

        friend struct AsyncPromise;
    private:
        __inline_always explicit Async(std::coroutine_handle<AsyncPromise> handle)
        {
            xTaskCreate([](void* pvParams)
            {
                auto handle = std::coroutine_handle<AsyncPromise>::from_address(pvParams);
                handle.resume();
                vTaskDelete(nullptr);
            }, "Async", Config::AsyncThreadStackSizeWords, handle.address(), Config::AsyncThreadPriority, nullptr);
        }
    };

    Async AsyncPromise::get_return_object()
    {
        return Async(std::coroutine_handle<AsyncPromise>::from_promise(*this));
    }
}
