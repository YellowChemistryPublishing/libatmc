#pragma once

#include <cmath>

namespace atmc
{
    template <size_t N, typename T = float>
    requires (N > 0)
    struct CenteringFilter
    {
        T values[N];
        size_t nextIndex = 0;
        T sum = 0;

        constexpr CenteringFilter() noexcept = default;

        inline void push(T value) noexcept
        {
            if (nextIndex >= N) [[likely]]
            {
                T valMin = std::numeric_limits<T>::max(), valMax = std::numeric_limits<T>::lowest();
                size_t iMin, iMax;
                for (size_t i = 0; i < N; i++)
                {
                    if (this->values[i] < valMin)
                    {
                        valMin = this->values[i];
                        iMin = i;
                    }
                    if (this->values[i] > valMax)
                    {
                        valMax = this->values[i];
                        iMax = i;
                    }
                }

                T avg = this->average();
                if (valMax - avg > avg - valMin)
                {
                    this->sum -= this->values[iMax];
                    this->values[iMax] = value;
                }
                else
                {
                    this->sum -= this->values[iMin];
                    this->values[iMin] = value;
                }
                this->sum += value;
            }
            else
            {
                this->values[nextIndex] = value;
                this->sum += value;
                ++nextIndex;
            }
        }
        inline T average() const noexcept
        {
            return this->sum / this->nextIndex;
        }
    };

    template <auto Alpha, typename T = float>
    struct IterativeFilter
    {
        T value = std::numeric_limits<T>::quiet_NaN();

        constexpr IterativeFilter() noexcept = default;

        inline void push(T newValue) noexcept
        {
            if (std::isnan(this->value)) [[unlikely]]
                this->value = newValue;
            else
                this->value = Alpha * newValue + (1 - Alpha) * this->value;
        }
        inline T average() const noexcept
        {
            return this->value;
        }
    };
} // namespace atmc