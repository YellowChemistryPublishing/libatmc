#include "rt.h"

#include <print>

// clang-format off
#include <FreeRTOS.h>
#include <../../freertos/include/task.h>
// clang-format on

#include <Config.h>
#include <LanguageSupport.h>
#include <TaskEx.h>

namespace atmc
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

    alignas(std::max_align_t) unsigned char TaskAllocator::stack[Config::TaskPromiseStackSize];
    TaskAllocator::ChunkHeader* TaskAllocator::stackTop;
} // namespace atmc

using namespace sys;
using namespace atmc;

void* __task_operator_new(size_t sz)
{
    return TaskAllocator::alloc(sz);
}
void __task_operator_delete(void* ptr)
{
    TaskAllocator::free(ptr);
}
void __launch_async(void* addr)
{
    xTaskCreate([](void* pvParams)
    {
        auto handle = std::coroutine_handle<async_promise>::from_address(pvParams);
        handle.resume();
        vTaskDelete(nullptr);
    }, "Async", atmc::Config::AsyncThreadStackSizeWords, addr, atmc::Config::AsyncThreadPriority, nullptr);
}
