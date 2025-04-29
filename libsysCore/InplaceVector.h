#pragma once

namespace atmc
{
    template <typename T, size_t Capacity>
    requires (Capacity > 0)
    class InplaceVector
    {
        union
        {
            T data[Capacity];
        };
        size_t _size = 0;
    public:
        inline InplaceVector()
        { }

        inline bool empty() const
        {
            return this->_size == 0;
        }
        inline size_t size() const
        {
            return this->_size;
        }
        consteval static size_t capacity()
        {
            return Capacity;
        }
        
        inline T& operator[](size_t index)
        {
            return this->data[index];
        }
        inline const T& operator[](size_t index) const
        {
            return this->data[index];
        }

        inline bool pushBack(T value)
        {
            if (this->_size >= Capacity)
                return false;
            new (&this->data[this->_size++]) T(std::move(value));
            return true;
        }
        inline void popBack()
        {
            if (this->_size > 0) [[likely]]
                this->data[this->_size--].~T();
        }
        inline void clear()
        {
            for (size_t i = 0; i < this->_size; ++i)
                this->data[i].~T();
            this->_size = 0;
        }

        inline T* find(const T& value)
        {
            for (size_t i = 0; i < this->_size; i++)
            {
                if (this->data[i] == value)
                    return &this->data[i];
            }
            return nullptr;
        }
    };
}