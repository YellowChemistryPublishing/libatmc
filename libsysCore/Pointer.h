#pragma once

#include <functional>
#include <utility>

namespace sys
{
    template <typename T>
    struct sc_ptr_b
    {
        constexpr sc_ptr_b() noexcept = default;
        constexpr sc_ptr_b(void* ptr) noexcept : _ptr(ptr)
        { }

        inline T& operator*() const noexcept
        {
            return *this->_ptr;
        }
        inline T* operator->() const noexcept
        {
            return this->_ptr;
        }

        inline T* move() noexcept
        {
            T* ret = this->_ptr;
            this->_ptr = nullptr;
            return ret;
        }
    private:
        T* _ptr = nullptr;
    };

    template <typename T>
    struct sc_ptr : sc_ptr_b<T>
    {
        using sc_ptr_b<T>::sc_ptr_b;
        inline ~sc_ptr()
        {
            delete this->_ptr;
        }
    };
    template <typename T>
    struct sc_ptr<T[]> : sc_ptr_b<T[]>
    {
        using sc_ptr_b<T>::sc_ptr_b;
        inline ~sc_ptr()
        {
            delete[] this->_ptr;
        }
    };

    template <typename Func>
    struct sc_act
    {
        constexpr sc_act(Func&& func) noexcept : func(std::forward<Func>(func))
        { }
        inline ~sc_act()
        {
            if (!this->abandon)
                std::invoke(func);
        }

        inline void release() noexcept
        {
            this->abandon = true;
        }
    private:
        [[no_unique_address]] Func func;
        bool abandon = false;
    };
} // namespace sys