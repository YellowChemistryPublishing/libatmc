#pragma once

#include <atomic>

namespace sys
{
    struct LockGuard;

    class SpinLock
    {
        std::atomic_flag locked = ATOMIC_FLAG_INIT;
    public:
        inline void lock()
        {
            while (locked.test_and_set(std::memory_order_acquire));
        }
        inline bool tryLock()
        {
            return !locked.test_and_set(std::memory_order_acquire);
        }
        inline void unlock()
        {
            locked.clear(std::memory_order_release);
        }

        inline LockGuard guard();
    };

    struct LockGuard
    {
        inline LockGuard(SpinLock& lock) : lock(lock)
        {
            this->lock.lock();
        }
        inline ~LockGuard()
        {
            this->lock.unlock();
        }
    private:
        SpinLock& lock;
    };

    LockGuard SpinLock::guard()
    {
        return LockGuard(*this);
    }
} // namespace sys