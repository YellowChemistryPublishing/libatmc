#pragma once

#include <string_view>

namespace sys
{
    template <size_t Capacity, typename CharType = char>
    class InplaceString
    {
        CharType buffer[Capacity + 1] { };
        size_t _size;
    public:
        constexpr InlineString() : size(0)
        {
            this->buffer[0] = 0;
        }
        constexpr InlineString(const char* cstr) : size(0)
        {
            size_t len = strlen(cstr);
            if (len > Capacity)
            {
                this->buffer[0] = 0;
                return;
            }
            
            memcpy(this->buffer, cstr, len + 1);
        }

        inline CharType* begin()
        {
            return this->buffer;
        }
        inline CharType* end()
        {
            return this->buffer + this->_size;
        }
    };
}
