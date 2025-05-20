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
        inline ManagedArray(ssz)
        {
            _assert_ctor_can_fail();
        }
        _const inline static Result<ManagedArray<T>> ctor(ssz len)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return nullptr;

            ManagedArray<T> ret;
            ret.data = new T[len];
            ret._length = len;
        }
        inline ManagedArray(ssz, T)
        {
            _assert_ctor_can_fail();
        }
        _const inline static Result<ManagedArray<T>> ctor(ssz len, T init)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return nullptr;

            ManagedArray<T> ret;
            sc_ptr<T[]> incomplete = new T[len];
            ret.forEachAssign(len, [&](ssz i) { incomplete[i] = init; });
            ret.data = incomplete.move();
            return ret;
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

        _const inline ManagedArray& operator=(const ManagedArray& other)
        {
            sc_ptr<T[]> incomplete = new T[other._length];
            this->forEachAssign(other._length, [&](ssz i) { incomplete[i] = other.data[i]; });
            delete[] this->data;
            this->data = incomplete.move();
            this->_length = other._length;
            return *this;
        }
        _const inline ManagedArray& operator=(ManagedArray&& other) noexcept
        {
            this->_length = 0;
            delete[] this->data;
            this->data = nullptr;
            swap(*this, other);
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
            if (ssz(0) <= index && index < this->_length) [[likely]]
                return (*this)[index, unsafe];
            else return nullptr;
        }
        _const inline Result<const T&> operator[](ssz index) const
        {
            if (ssz(0) <= index && index < this->_length) [[likely]]
                return (*this)[index, unsafe];
            else return nullptr;
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

        _const inline bool isEmpty() const noexcept
        {
            return this->_length == 0;
        }
        _const inline ssz length() const noexcept
        {
            return this->_length;
        }
        _const inline size_t size() const noexcept
        {
            return this->length();
        }
        _const inline ssz capacity() const noexcept
        {
            return this->_length;
        }
        inline void clear()
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
            for (ssz i = 0; i < len; i++) std::invoke(withIndex, assign, i);
            this->data = assign;
            this->_length = len;
        }
    };
} // namespace sys
