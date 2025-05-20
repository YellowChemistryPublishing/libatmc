#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace sys
{
    template <typename T>
    struct sc_ptr_b
    {
        constexpr sc_ptr_b() noexcept = default;
        constexpr sc_ptr_b(void* ptr) noexcept : _ptr(ptr)
        { }
        constexpr sc_ptr_b(const sc_ptr_b&) = delete;
        constexpr sc_ptr_b(sc_ptr_b&& other) noexcept : _ptr(other._ptr)
        {
            swap(*this, other);
        }
        constexpr ~sc_ptr_b()
        {
            delete this->_ptr;
        }

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

        friend inline void swap(sc_ptr_b& a, sc_ptr_b& b) noexcept
        {
            std::swap(a._ptr, b._ptr);
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
        constexpr sc_act(const sc_act&) = delete;
        constexpr sc_act(sc_act&& other) noexcept
        {
            swap(*this, other);
        }
        inline ~sc_act()
        {
            if (!this->released)
                std::invoke(func);
        }

        inline void release() noexcept
        {
            this->released = true;
        }

        friend inline void swap(sc_act& a, sc_act& b) noexcept(std::is_nothrow_swappable_v<sc_act>)
        {
            std::swap(a.func, b.func);
            std::swap(a.released, b.released);
        }
    private:
        [[no_unique_address]] Func func;
        bool released = false;
    };
} // namespace sys