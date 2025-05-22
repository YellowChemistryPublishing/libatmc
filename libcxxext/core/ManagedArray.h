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
        [[nodiscard]] _const inline static Result<ManagedArray<T>> ctor(ssz len)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return nullptr;

            ManagedArray<T> ret;
            ret.data = new T[len];
            ret._length = len;
            return ret;
        }
        inline ManagedArray(ssz, T)
        {
            _assert_ctor_can_fail();
        }
        [[nodiscard]] _const inline static Result<ManagedArray<T>> ctor(ssz len, T init)
        {
            if (std::cmp_less(len, 0) || std::cmp_greater(len, std::numeric_limits<size_t>::max()))
                return nullptr;

            ManagedArray<T> ret;
            sc_ptr<T[]> incomplete = new T[len];
            for (auto it = &*incomplete; it != &*incomplete + len; ++it) *it = init;
            ret.data = incomplete.move();
            ret._length = len;
            return ret;
        }
        inline ManagedArray(std::initializer_list<T> init)
        {
            sc_ptr<T[]> incomplete = new T[init.size()];
            for (auto it1 = &*incomplete, it2 = init.begin(); it1 != &*incomplete + init.size(); ++it1, ++it2)
                *it1 = *it2;
            this->data = incomplete.move();
            this->_length = init.size();
        }
        inline ManagedArray(const ManagedArray& other)
        {
            sc_ptr<T[]> incomplete = new T[other._length];
            for (auto it1 = &*incomplete, it2 = other.begin(); it2 != other.end(); ++it1, ++it2)
                *it1 = *it2;
            this->data = incomplete.move();
            this->_length = other._length;
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
            for (auto it1 = &*incomplete, it2 = other.begin(); it2 != other.end(); ++it1, ++it2)
                *it1 = *it2;
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

        _const inline const T& operator[](ssz index, unsafe_tag) const
        {
            return this->data[index];
        }
        _const inline T& operator[](ssz index, unsafe_tag)
        {
            return _invoke_const_member_overload(operator[](index, unsafe), _asc);
        }
        _const inline Result<const T&> operator[](ssz index) const
        {
            if (ssz(0) <= index && index < this->_length) [[likely]]
                return (*this)[index, unsafe];
            else
                return nullptr;
        }
        _const inline Result<T&> operator[](ssz index)
        {
            auto ret = _as(const ManagedArray<T>*, this)->operator[](index);
            return _asr(Result<T&>&, ret);
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
            using std::swap;

            swap(a.data, b.data);
            swap(a._length, b._length);
        }
    private:
        T* data = nullptr;
        ssz _length = 0;

        inline ManagedArray()
        { }
    };
} // namespace sys
