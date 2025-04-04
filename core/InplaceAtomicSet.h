#pragma once

#include <atomic>

namespace atmc
{
    /// @brief An in-place set of atomic values that can be exchanged.
    /// @tparam T Type of value to store.
    /// @tparam Capacity Maximum number of values that can be stored.
    template <typename T, size_t Capacity>
    requires (Capacity > 0)
    class InplaceAtomicSet
    {
        std::atomic<T> data[Capacity];
    public:
        /// @brief Default constructor.
        inline InplaceAtomicSet() = default;
        /// @brief Initialize all elements with the same value.
        /// @param init Value to set.
        inline InplaceAtomicSet(T init)
        {
            for (size_t i = 0; i < Capacity; i++)
                this->data[i].store(init);
        }
        
        /// @brief Exchange the value of the first element that matches the expected value.
        /// @param from Value to exchange with.
        /// @param to Value to exchange to.
        /// @return Whether an element was exchanged.
        bool exchangeWeak(T from, T to)
        {
            for (size_t i = 0; i < Capacity; i++)
            {
                T expected = from;
                if (this->data[i].compare_exchange_weak(expected, to))
                    return true;
            }
            return false;
        }
        /// @brief Exchange the value of the first element that matches the expected value.
        /// @param from Value to exchange with.
        /// @param to Value to exchange to.
        /// @return Whether an element was exchanged.
        bool exchange(T from, T to)
        {
            for (size_t i = 0; i < Capacity; i++)
            {
                T expected = from;
                if (this->data[i].compare_exchange_strong(expected, to))
                    return true;
            }
            return false;
        }
    };
}