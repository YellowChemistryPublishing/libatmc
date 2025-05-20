#pragma once

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <limits>
#include <utility>

#include <LanguageSupport.h>
#include <Pointer.h>
#include <Result.h>

namespace sys
{
    template <typename T>
    struct ManagedArray
    {
        inline ManagedArray(ssz len)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return;

            this->data = new T[len];
            this->_length = len;
        }
        inline ManagedArray(ssz len, T init)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return;

            sc_ptr<T[]> incomplete = new T[len];
            this->forEachAssign(len, [&](ssz i) { incomplete[i] = init; });
            this->data = incomplete.move();
        }
        inline ManagedArray(std::initializer_list<T> init)
        {
            sc_ptr<T[]> incomplete = new T[init.size()];
            auto it = init.begin();
            this->forEachAssign(init.size(), [&](ssz i)
            {
                incomplete[i] = *it;
                ++it;
            });
            this->data = incomplete.move();
        }
        inline ManagedArray(const ManagedArray& other)
        {
            sc_ptr<T[]> incomplete = new T[other._length];
            this->forEachAssign(other._length, [&](ssz i) { incomplete[i] = other.data[i]; });
            this->data = incomplete.move();
        }
        inline ManagedArray(ManagedArray&& other) noexcept
        {
            swap(*this, other);
        }
        inline ~ManagedArray()
        {
            delete[] this->data;
        }

        inline ManagedArray& operator=(const ManagedArray& other)
        {
            sc_ptr<T[]> incomplete = new T[other._length];
            this->forEachAssign(other._length, [&](ssz i) { incomplete[i] = other.data[i]; });
            delete[] this->data;
            this->data = incomplete.move();
            this->_length = other._length;
            return *this;
        }
        inline ManagedArray& operator=(ManagedArray&& other)
        {
            this->_length = 0;
            delete[] this->data;
            this->data = nullptr;
            std::swap(*this, other);
            return *this;
        }

        _const inline T& operator[](ssz index, unsafe_tag)
        {
            return this->data[index];
        }
        _const inline const T& operator[](ssz index, unsafe_tag) const
        {
            return this->data[index];
        }
        _const inline Result<T&> operator[](ssz index)
        {
            return this->data[index];
        }
        _const inline Result<const T&> operator[](ssz index) const
        {
            return this->data[index];
        }

        _const inline const T* cbegin() const noexcept
        {
            return this->data;
        }
        _const inline const T* cend() const noexcept
        {
            return this->data + this->_length;
        }
        _const inline const T* begin() const noexcept
        {
            return this->cbegin();
        }
        _const inline const T* end() const noexcept
        {
            return this->cend();
        }
        _const inline T* begin() noexcept
        {
            return this->data;
        }
        _const inline T* end() noexcept
        {
            return this->data + this->_length;
        }

        _const inline ssz length()
        {
            return this->_length;
        }
        inline size_t size() const
        {
            return this->length();
        }
        inline void clear() noexcept
        {
            delete[] this->data;
            this->data = nullptr;
            this->_length = 0;
        }

        friend inline void swap(sys::ManagedArray<T>& a, sys::ManagedArray<T>& b) noexcept
        {
            std::swap(a.data, b.data);
            std::swap(a._length, b._length);
        }
    private:
        T* data = nullptr;
        ssz _length = 0;

        inline void forEachAssign(T* assign, ssz len, auto&& withIndex)
        {
            sc_ptr<T[]> incomplete = assign;
            for (ssz i = 0; i < len; i++) std::invoke(withIndex, &*incomplete, i);
            this->data = incomplete.move();
            this->_length = len;
        }
    };
} // namespace sys
