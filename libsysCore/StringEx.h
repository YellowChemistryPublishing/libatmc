#pragma once

#include <algorithm>
#include <concepts>
#include <cxxsup.h>
#include <functional>
#include <span>

#include <LanguageSupport.h>
#include <ManagedArray.h>

namespace sys
{
    template <std::integral CharType = char8_t, bool DynamicExtent = true, ssz StaticCapacity = 4>
    requires (StaticCapacity > 0 && std::is_trivially_constructible_v<CharType> && std::is_trivially_copyable_v<CharType>)
    struct String
    {
        template <IEnumerable<String> Container>
        static Result<String> concat(Container&& container)
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
        inline String(const String<CharType, OtherDynamicExtent, OtherStaticCapacity>& other)
        {
            if (other.isDynamic)
            {
                this->forEachAssign(other._length, [&](CharType* buf, ssz i) { buf[i] = other.dataDynamic.buf[i]; }, [&, this]
                {
                    this->dataDynamic.buf = ManagedArray<CharType>(other.dataDynamic.capacity);
                    this->dataDynamic.capacity = other.dataDynamic.capacity;
                    this->isDynamic = true;
                    return this->dataDynamic.buf.begin();
                });
            }
            else
            {
                this->forEachAssign(other._length, [&](CharType* buf, ssz i) { buf[i] = other.dataStatic[i]; }, [&, this]
                {
                    this->isDynamic = false;
                    return this->dataStatic;
                });
            }
        }
        inline String(String<CharType, DynamicExtent, StaticCapacity>&& other)
        {
            swap(*this, other);
        }
        inline ~String()
        {
            this->dealloc();
        }

        friend inline auto operator<=>(const String& a, const String& b)
        {
            return std::lexicographical_compare_three_way(a.begin(), a.end(), b.begin(), b.end());
        }

        inline CharType& operator[](ssz i, unsafe_tag) noexcept
        {
            if (this->isDynamic)
                return this->dataDynamic.buf[i];
            else
                return this->dataStatic[i];
        }
        inline CharType operator[](ssz i, unsafe_tag) const noexcept
        {
            if (this->isDynamic)
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
            if (this->isDynamic)
                return this->dataDynamic.buf.begin();
            else
                return this->dataStatic;
        }
        inline const CharType* cend() const noexcept
        {
            if (this->isDynamic)
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
            if (this->isDynamic)
                return this->dataDynamic.buf.begin();
            else
                return this->dataStatic;
        }
        inline CharType* end() noexcept
        {
            if (this->isDynamic)
                return this->dataDynamic.buf.end();
            else
                return this->dataStatic + this->_length;
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

        friend inline void swap(sys::String<CharType, DynamicExtent, StaticCapacity>& a, sys::String<CharType, DynamicExtent, StaticCapacity>& b) noexcept
        {
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
        bool isDynamic = false;
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
                    this->isDynamic = true;
                    return this->dataDynamic.buf.begin();
                }
                else
                {
                    this->isDynamic = false;
                    return this->dataStatic;
                }
            }

            if (capacity <= StaticCapacity)
                return this->dataStatic;

            return nullptr;
        }
        inline void dealloc()
        {
            if constexpr (DynamicExtent)
            {
                if (this->isDynamic)
                    this->dataDynamic.buf.~ManagedArray();
            }
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
