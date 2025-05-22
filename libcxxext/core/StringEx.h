#pragma once

#include <algorithm>
#include <concepts>
#include <cxxsup.h>
#include <functional>
#include <span>

#include <LanguageSupport.h>
#include <ManagedArray.h>
#include <string_view>

namespace sys
{
    template <std::integral CharType = char8_t, bool DynamicExtent = true, ssz StaticCapacity = 4>
    requires (StaticCapacity > 0 && std::is_trivially_constructible_v<CharType> && std::is_trivially_copyable_v<CharType>)
    struct String
    {
        template <IEnumerable<String> Container>
        _const static Result<String> concat(Container&& container)
        {
            ssz totalLength = 0;
            for (auto& str : container) totalLength += str.length();

            String ret;
            auto to = ret.alloc(nr2i64(totalLength + 1));
            if (!to)
                return nullptr;

            for (auto& str : container)
            {
                std::copy(str.begin(), str.end(), to);
                to += str.length();
            }
            *to = 0;

            ret._length = totalLength;
            return ret;
        }

        inline String()
        {
            this->dataStatic[0] = 0;
        }
        inline String(CharType c, ssz repeat)
        {
            this->forEachAssign(repeat, [&](CharType* buf, ssz i) { buf[i] = c; }, [&, this] { return this->alloc(nr2i64(repeat + 1)); });
        }
        inline String(const std::span<CharType> str)
        {
            this->forEachAssign(str.size(), [&](CharType* buf, ssz i) { buf[i] = str[i]; }, [&, this] { return this->alloc(nr2i64(str.size() + 1)); });
        }
        inline String(const CharType* str)
        {
            ssz len = 0;
            while (str[len]) ++len;
            this->forEachAssign(len, [&](CharType* buf, ssz i) { buf[i] = str[i]; }, [&, this] { return this->alloc(nr2i64(len + 1)); });
        }
        template <bool OtherDynamicExtent, ssz OtherStaticCapacity>
        inline String(const String<CharType, OtherDynamicExtent, OtherStaticCapacity>&)
        {
            _assert_ctor_can_fail();
        }
        template <bool OtherDynamicExtent, ssz OtherStaticCapacity>
        inline static Result<String<CharType, DynamicExtent, StaticCapacity>> ctor(const String<CharType, OtherDynamicExtent, OtherStaticCapacity>& other)
        {
            if (other.isDynamic())
            {
                String<CharType, DynamicExtent, StaticCapacity> ret;
                ret->forEachAssign(other._length, [&](CharType* buf, ssz i) { buf[i] = other.dataDynamic.buf[i]; }, [&]
                {
                    ret->dataDynamic.buf = ManagedArray<CharType>(other.dataDynamic.capacity);
                    ret->dataDynamic.capacity = other.dataDynamic.capacity;
                    ret->_isDynamic = true;
                    return ret->dataDynamic.buf.begin();
                });
            }
            else if (other._length < StaticCapacity)
            {
                String<CharType, DynamicExtent, StaticCapacity> ret;
                ret.forEachAssign(other._length, [&](CharType* buf, ssz i) { buf[i] = other.dataStatic[i]; }, [&]
                {
                    ret._isDynamic = false;
                    return ret.dataStatic;
                });
            }
            else
                return nullptr;
        }
        inline String(String<CharType, DynamicExtent, StaticCapacity>&& other)
        {
            swap(*this, other);
        }
        inline ~String()
        {
            this->dealloc();
        }

        inline operator std::basic_string_view<CharType>() const noexcept
        {
            if (this->isDynamic())
                return std::basic_string_view<CharType>(this->dataDynamic.buf.begin(), this->_length);
            else
                return std::basic_string_view<CharType>(this->dataStatic, this->_length);
        }

        friend inline auto operator<=>(const String& a, const String& b)
        {
            return std::lexicographical_compare_three_way(a.begin(), a.end(), b.begin(), b.end());
        }

        inline CharType& operator[](ssz i, unsafe_tag) noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf[i];
            else
                return this->dataStatic[i];
        }
        inline CharType operator[](ssz i, unsafe_tag) const noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf[i];
            else
                return this->dataStatic[i];
        }
        inline Result<CharType&> operator[](ssz i)
        {
            if (ssz(0) <= i && i < this->_length)
                return (*this)[i, unsafe];
            else
                return nullptr;
        }
        inline Result<CharType> operator[](ssz i) const
        {
            if (ssz(0) <= i && i < this->_length)
                return (*this)[i, unsafe];
            else
                return nullptr;
        }

        inline const CharType* cbegin() const noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf.begin();
            else
                return this->dataStatic;
        }
        inline const CharType* cend() const noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf.end();
            else
                return this->dataStatic + this->_length;
        }
        inline const CharType* begin() const noexcept
        {
            return this->cbegin();
        }
        inline const CharType* end() const noexcept
        {
            return this->cend();
        }
        inline CharType* begin() noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf.begin();
            else
                return this->dataStatic;
        }
        inline CharType* end() noexcept
        {
            if (this->isDynamic())
                return this->dataDynamic.buf.end();
            else
                return this->dataStatic + this->_length;
        }

        constexpr bool isDynamic() const noexcept
        {
            if constexpr (DynamicExtent)
                return this->_isDynamic;
            else
                return false;
        }
        inline bool isEmpty() const
        {
            return this->_length == 0;
        }
        inline ssz length() const
        {
            return this->_length;
        }
        inline size_t size() const
        {
            return _asi(size_t, this->length());
        }
        inline ssz capacity() const
        {
            if (this->isDynamic())
                return this->dataDynamic.capacity;
            else
                return StaticCapacity;
        }

        inline bool contains(CharType c) const
        {
            return this->find(c);
        }
        inline bool contains(const std::basic_string_view<CharType> str) const
        {
            return this->find(str);
        }
        inline bool startsWith(const std::basic_string_view<CharType> str) const
        {
            if (this->length() < str.length())
                return false;
            return std::equal(this->cbegin(), this->cbegin() + str.length(), str.begin());
        }
        inline bool endsWith(const std::basic_string_view<CharType> str) const
        {
            if (this->length() < str.length())
                return false;
            return std::equal(this->cend() - str.length(), this->cend(), str.begin());
        }
        inline const CharType* find(CharType c) const
        {
            for (auto it = this->cbegin(); it != this->cend(); ++it)
            {
                if (*it == c)
                    return it;
            }
            return nullptr;
        }
        inline const CharType* find(const std::basic_string_view<CharType> str) const
        {
            for (auto it = this->cbegin(); it != this->cend() - str.length() + 1; ++it)
            {
                if (std::equal(it, it + str.length(), str.begin()))
                    return it;
            }
            return nullptr;
        }
        inline CharType* find(CharType c)
        {
            for (auto it = this->begin(); it != this->end(); ++it)
            {
                if (*it == c)
                    return it;
            }
            return nullptr;
        }

        inline sz hashCode()
        {
            return std::hash(_as(std::basic_string_view<CharType>, *this))();
        }
        friend inline void swap(sys::String<CharType, DynamicExtent, StaticCapacity>& a, sys::String<CharType, DynamicExtent, StaticCapacity>& b) noexcept
        {
            using std::swap;

            if (&a != &b) [[likely]]
                std::swap_ranges(_asr(byte*, &a), _asr(byte*, &a) + sizeof(a), _asr(byte*, &b));
        }
    private:
        union
        {
            struct
            {
                ManagedArray<CharType> buf;
                ssz capacity = 0;
            } dataDynamic;
            CharType dataStatic[sz(StaticCapacity)];
        };
        bool _isDynamic = false;
        ssz _length = 0;

        inline String(unsafe_tag)
        { }

        [[nodiscard]] inline CharType* alloc(ssz capacity)
        {
            if constexpr (DynamicExtent)
            {
                if (capacity > StaticCapacity)
                {
                    this->dataDynamic.buf = ManagedArray<CharType>(capacity);
                    this->dataDynamic.capacity = capacity;
                    this->_isDynamic = true;
                    return this->dataDynamic.buf.begin();
                }
                else
                {
                    this->_isDynamic = false;
                    return this->dataStatic;
                }
            }

            if (capacity <= StaticCapacity)
                return this->dataStatic;

            return nullptr;
        }
        inline void dealloc()
        {
            if (this->isDynamic())
                this->dataDynamic.buf.~ManagedArray();
        }
        inline bool forEachAssign(ssz len, auto&& withIndex, auto&& createFetchBuffer)
        {
            if (CharType* buf = std::invoke_r<CharType*>(createFetchBuffer))
            {
                for (ssz i = 0; i < len; i++) std::invoke(withIndex, buf, i);
                buf[len] = 0;
                this->_length = len;
                return true;
            }
            return false;
        }
    };

    using string = sys::String<char8_t>;
    using str16 = sys::String<char16_t>;
    using str32 = sys::String<char32_t>;
    using cstr = sys::String<char>;
    using wstr = sys::String<wchar_t>;
} // namespace sys
