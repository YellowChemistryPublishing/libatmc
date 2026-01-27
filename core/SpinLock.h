#pragma once

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
            while (this->flag.test_and_set(std::memory_order_acquire))
            { }
        }
        void unlock() { this->flag.clear(std::memory_order_release); }
    private:
        std::atomic_flag flag;
    };

    struct LockGuard final
    {
        explicit LockGuard(SpinLock& lock) : lock(lock) { this->lock.lock(); }
        LockGuard(const LockGuard&) = delete;
        LockGuard(LockGuard&&) = delete;
        ~LockGuard() { this->lock.unlock(); }

        LockGuard& operator=(const LockGuard&) = delete;
        LockGuard& operator=(LockGuard&&) = delete;
    private:
        SpinLock& lock; // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    };
} // namespace atmc
