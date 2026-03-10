#pragma once

/// @file

#include <atomic>

namespace atmc
{
    struct SpinLock final
    {
        SpinLock() = default;
        SpinLock(const SpinLock&) = delete;
        SpinLock(SpinLock&&) = delete;
        ~SpinLock() = default;

        SpinLock& operator=(const SpinLock&) = delete;
        SpinLock& operator=(SpinLock&&) = delete;

        void lock()
        {
        KeepWaiting:
            while (this->flag.test(std::memory_order_acquire))
                if (this->flag.test_and_set(std::memory_order_acquire))
                    goto KeepWaiting;
        }
        void unlock() { this->flag.clear(std::memory_order_release); }
    private:
        std::atomic_flag flag;
    };
} // namespace atmc
