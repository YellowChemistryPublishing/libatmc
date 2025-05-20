#pragma once

#include <coroutine>

// clang-format off
#include <FreeRTOS.h>
#include <../../freertos/include/task.h>
// clang-format on

#include <Config.h>
#include <LanguageSupport.h>

namespace sys
{
    /// @brief
    /// @note Static class.
    class TaskAllocator
    {
        //     v Lazy.
        struct alignas(max_align_t) ChunkHeader
        {
            ChunkHeader* prevChunk;
            size_t size;
            bool isFree = false;

            inline unsigned char* data() noexcept
            {
                return reinterpret_cast<unsigned char*>(this) + sizeof(ChunkHeader);
            }
            inline void* next() noexcept
            {
                return reinterpret_cast<unsigned char*>(this) + sizeof(ChunkHeader) + this->size;
            }
        };

        /* Lazy.          */
        alignas(max_align_t) static unsigned char stack[atmc::Config::TaskPromiseStackSize];
        static ChunkHeader* stackTop;
    public:
        TaskAllocator() = delete;

        inline static void* alloc(size_t sz) noexcept
        {
            size_t chunkSize = (sz + alignof(std::max_align_t) - 1) & -alignof(std::max_align_t);
            if ((!TaskAllocator::stackTop ? TaskAllocator::stack : _as(unsigned char*, TaskAllocator::stackTop->next())) + sizeof(ChunkHeader) + chunkSize >=
                TaskAllocator::stack + atmc::Config::TaskPromiseStackSize) [[unlikely]]
                return nullptr;
            ChunkHeader* header = [&]
            {
                if (!TaskAllocator::stackTop)
                    return new(TaskAllocator::stack) ChunkHeader(nullptr, chunkSize);
                else
                    return new(TaskAllocator::stackTop->next()) ChunkHeader(TaskAllocator::stackTop, chunkSize);
            }();
            TaskAllocator::stackTop = header;
            return header->data();
        }
        inline static void free(void* ptr) noexcept
        {
            if (!ptr) [[unlikely]]
                return;

            ChunkHeader* header = _asr(ChunkHeader*, ptr) - 1;
            if (TaskAllocator::stackTop == header)
            {
                do TaskAllocator::stackTop = TaskAllocator::stackTop->prevChunk;
                while (TaskAllocator::stackTop && TaskAllocator::stackTop->isFree);
            }
            else
                header->isFree = true;
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

        _inline_always constexpr bool await_ready() const noexcept
        {
            return !this->handle || this->handle.done();
        }
        _inline_never std::coroutine_handle<> await_suspend(std::coroutine_handle<> parent)
        {
            this->handle.promise().continuation = parent;
            taskYIELD();
            return this->handle;
        }
        _inline_always constexpr T await_resume() const noexcept(std::is_same<T, void>::value)
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

        _inline_always constexpr bool await_ready() const noexcept
        {
            return false;
        }
        _inline_never std::coroutine_handle<> await_suspend(std::coroutine_handle<>) noexcept
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

        inline static void* operator new(size_t sz) noexcept
        {
            return TaskAllocator::alloc(sz);
        }
        inline static void operator delete(void* ptr) noexcept
        {
            TaskAllocator::free(ptr);
        }

        _inline_always constexpr std::suspend_always initial_suspend() const noexcept
        {
            return std::suspend_always();
        }

        _inline_always Task<T> get_return_object()
        {
            return Task<T>(std::coroutine_handle<TaskPromise<T>>::from_promise(*static_cast<TaskPromise<T>*>(this)));
        }

        [[noreturn]] inline static Task<T> get_return_object_on_allocation_failure()
        {
            __throw(std::bad_alloc());
        }
        _inline_always void unhandled_exception()
        {
            this->exception = std::current_exception();
        }
    };
    template <typename T>
    struct TaskPromise final : public TaskPromiseCore<T>
    {
        alignas(T) unsigned char value[sizeof(T)];

        _inline_always TaskFinalAwaiter<T> final_suspend() noexcept
        {
            return TaskFinalAwaiter<T> { std::coroutine_handle<TaskPromise<T>>::from_promise(*this) };
        }

        template <typename ReturnType>
        _inline_always constexpr void return_value(ReturnType&& ret)
        {
            new(this->value) T(std::forward<ReturnType>(ret));
        }
    };
    template <>
    struct TaskPromise<void> final : public TaskPromiseCore<void>
    {
        _inline_always TaskFinalAwaiter<void> final_suspend() noexcept
        {
            return TaskFinalAwaiter<void> { std::coroutine_handle<TaskPromise<void>>::from_promise(*this) };
        }

        _inline_always constexpr void return_void() const noexcept
        { }
    };

    template <typename T = void>
    class [[nodiscard]] Task final
    {
    public:
        using promise_type = TaskPromise<T>;

        constexpr static uint32_t MaxDelay = HAL_MAX_DELAY;

        _inline_always ~Task()
        {
            if (this->handle)
                this->handle.destroy();
        }

        _inline_always TaskAwaiter<T> operator co_await()
        {
            return TaskAwaiter<T>(this->handle);
        }

        inline static auto yield()
        requires (std::is_same<T, void>::value)
        {
            struct
            {
                _inline_always constexpr bool await_ready() const noexcept
                {
                    taskYIELD();
                    return true;
                }
                _inline_always void await_suspend(std::coroutine_handle<>) const noexcept
                { }
                _inline_always constexpr void await_resume() const noexcept
                { }
            } ret;
            return ret;
        }
        inline static Task<void> delay(uint32_t ms)
        requires (std::is_same<T, void>::value)
        {
            uint32_t from = xTaskGetTickCount();
            while (pdTICKS_TO_MS(xTaskGetTickCount() - from) < ms) co_await Task<>::yield();
        }
        template <typename Pred>
        inline static Task<void> waitUntil(Pred&& func)
        {
            if constexpr (!std::convertible_to<decltype(func()), bool>)
                while (!co_await func());
            else
                while (!func()) co_await Task<>::yield();
        }

        friend struct sys::TaskPromiseCore<T>;
    private:
        std::coroutine_handle<TaskPromise<T>> handle;

        _inline_always explicit Task(std::coroutine_handle<TaskPromise<T>> handle) : handle(handle)
        { }
    };

    struct Async;

    struct AsyncPromise
    {
        consteval AsyncPromise() noexcept = default;
        _inline_always constexpr ~AsyncPromise() = default;

        _inline_always Async get_return_object();

        _inline_always constexpr std::suspend_always initial_suspend() const noexcept
        {
            return std::suspend_always();
        }
        _inline_always constexpr std::suspend_never final_suspend() const noexcept
        {
            return std::suspend_never();
        }

        inline void unhandled_exception() noexcept
        {
            std::rethrow_exception(std::current_exception());
        }
        _inline_always constexpr void return_void() const noexcept
        { }
    };

    struct Async
    {
        using promise_type = AsyncPromise;

        friend struct AsyncPromise;
    private:
        _inline_always explicit Async(std::coroutine_handle<AsyncPromise> handle)
        {
            xTaskCreate(
                [](void* pvParams)
                {
                    auto handle = std::coroutine_handle<AsyncPromise>::from_address(pvParams);
                    handle.resume();
                    vTaskDelete(nullptr);
                },
                "Async", atmc::Config::AsyncThreadStackSizeWords, handle.address(), atmc::Config::AsyncThreadPriority, nullptr);
        }
    };

    Async AsyncPromise::get_return_object()
    {
        return Async(std::coroutine_handle<AsyncPromise>::from_promise(*this));
    }
} // namespace sys

#define __async(void) ::sys::Async
