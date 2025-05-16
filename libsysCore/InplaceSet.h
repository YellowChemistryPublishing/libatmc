#pragma once

namespace sys
{
    template <typename T, size_t Capacity>
    requires (Capacity > 0)
    class InplaceSet
    {
        T data[Capacity];
        size_t _size = 0;
    public:
        inline InplaceSet() = default;

        inline size_t size() const
        {
            return this->_size;
        }
        consteval static size_t capacity()
        {
            return Capacity;
        }
        
        inline bool tryInsert(T value)
        {
            if (this->_size == Capacity) [[unlikely]]
                return false;
            
            this->data[this->_size++] = std::move(value);
            return true;
        }
        inline bool tryErase(T value)
        {
            for (size_t i = 0; i < this->_size; i++)
            {
                if (this->data[i] == value)
                {
                    this->data[i] = std::move(this->data[--this->_size]);
                    return true;
                }
            }
            return false;
        }
        inline bool contains(T value)
        {
            for (size_t i = 0; i < this->_size; i++)
            {
                if (this->data[i] == value)
                    return true;
            }
            return false;
        }
    };
}