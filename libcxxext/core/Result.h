#pragma once

#include <algorithm>
#include <coroutine>
#include <cstddef>
#include <print>
#include <type_traits>

#include <CompilerWarnings.h>
#include <Exception.h>
#include <LanguageSupport.h>

#define __fence_result_return(rValRef, out) \
    do                                      \
    {                                       \
        auto __result = rValRef;            \
        if (!__result)                      \
            return __result.err();          \
        out = __result.move();              \
    }                                       \
    while (false)
#define __fence_result_co_return(rValRef, out) \
    do                                         \
    {                                          \
        auto __result = rValRef;               \
        if (!__result)                         \
            co_return __result.err();          \
        out = __result.move();                 \
    }                                          \
    while (false)

namespace sys
{
    enum ResultStatus : u8
    {
        Ok,
        Error,
        Empty
    };

    template <typename T, typename Err>
    struct ResultAwaiter;

    template <typename T>
    using ResultStorageType = std::conditional_t<!std::is_reference_v<T>, std::remove_const_t<T>, std::remove_reference_t<T>*>;

    template <template <typename T, typename Err> class Result, typename T, typename Err>
    struct ResultCommon
    {
        /// @brief Whether the result is good.
        constexpr operator bool() const noexcept
        {
            using ThisType = TypeWithQualRefFrom<ResultType, std::remove_pointer_t<decltype(this)>>*;
            return _as(ThisType, this)->status == ResultStatus::Ok;
        }

        _inline_always ResultAwaiter<T, Err> operator co_await()
        {
            return ResultAwaiter<T, Err>(*this);
        }

        /// @brief Takes the value if the result is good.
        /// @return The value held by the result.
        constexpr T move()
        {
            using ThisType = TypeWithQualRefFrom<ResultType, std::remove_pointer_t<decltype(this)>>*;

            _fence_contract_enforce(_as(ThisType, this)->status == ResultStatus::Ok && "Taking value for a bad result!");
            _as(ThisType, this)->status = ResultStatus::Empty;
            if constexpr (std::is_reference_v<T>)
                return *_as(ThisType, this)->value;
            else
                return _as(ThisType, this)->value;
        }
        constexpr T expect()
        {
            using ThisType = TypeWithQualRefFrom<ResultType, std::remove_pointer_t<decltype(this)>>*;

            if (std::exchange(_as(ThisType, this)->status, ResultStatus::Empty) == ResultStatus::Ok)
            {
                if constexpr (std::is_reference_v<T>)
                    return *_as(ThisType, this)->value;
                else
                    return _as(ThisType, this)->value;
            }
            else
                _throw(ContractViolationException("Result is not ok!"));
        }
    private:
        using ResultType = Result<T, Err>;
    };

    /// @brief A result type that can hold either a value or an error.
    /// @tparam T The type of the value to hold.
    /// @tparam Err The type of the error to hold.
    template <typename T, typename Err = void>
    struct Result : ResultCommon<Result, T, Err>
    {
        /// @brief Constructs a result with a value.
        /// @param value Value to move into the result.
        constexpr Result(T&& value)
        {
            if constexpr (std::is_reference_v<T>)
                this->value = &value;
            else
                new(&this->value) ResultStorageType<T>(std::forward<T>(value));
            this->status = ResultStatus::Ok;
        }
        /// @brief Constructs a result with an error.
        /// @param error Error to move into the result.
        constexpr Result(Err&& error) : status(ResultStatus::Error)
        {
            if constexpr (std::is_reference_v<T>)
                this->error = &error;
            else
                new(&this->error) ResultStorageType<Err>(std::forward<Err>(error));
        }
        constexpr Result(Result&& other)
        {
            swap(*this, other);
        }
        constexpr ~Result() noexcept(false)
        {
            switch (this->status)
            {
            case ResultStatus::Ok:
                if constexpr (!std::is_reference_v<T>)
                    this->value.~ResultStorageType<T>();
                break;
            case ResultStatus::Error:
                if constexpr (!std::is_reference_v<Err>)
                    this->error.~ResultStorageType<Err>();
                break;
            }
        }

        constexpr Err err()
        {
            _fence_contract_enforce(this->status == ResultStatus::Error && "Taking error for a good or empty result!");
            this->status = ResultStatus::Empty;
            return this->template obtainStorage<Err>();
        }

        constexpr void swap(Result& a, Result& b)
        {
            using std::swap;

            if (&a != &b && (a.status == ResultStatus::Ok || a.status == ResultStatus::Error || b.status == ResultStatus::Ok || b.status == ResultStatus::Error))
                std::swap_ranges(_asr(byte*, &a.value), _asr(byte*, &a.value) + std::max(sizeof(ResultStorageType<T>), sizeof(ResultStorageType<Err>)), _asr(byte*, &b.value));
            swap(a.status, b.status);
        }

        friend struct sys::ResultCommon<Result, T, Err>;
    private:
        union
        {
            ResultStorageType<T> value;
            ResultStorageType<Err> error;
        };
        ResultStatus status = ResultStatus::Empty;
    };

    template <typename T>
    struct Result<T, void> : ResultCommon<Result, T, void>
    {
        /// @brief Constructs a result with a value.
        /// @param value Value to move into the result.
        constexpr Result(T&& value)
        {
            if constexpr (std::is_reference_v<T>)
                this->value = &value;
            else
                new(&this->value) ResultStorageType<T>(std::forward<T>(value));
            this->status = ResultStatus::Ok;
        }
        constexpr Result(std::nullptr_t) : status(ResultStatus::Error)
        { }
        constexpr Result(Result&& other)
        {
            swap(*this, other);
        }
        constexpr ~Result()
        {
            if constexpr (!std::is_reference_v<T>)
                if (this->status == ResultStatus::Ok)
                    _asr(ResultStorageType<T>*, this->storage)->~T();
        }

        constexpr void swap(Result& a, Result& b)
        {
            using std::swap;

            if (&a != &b && (a.status == ResultStatus::Ok || b.status == ResultStatus::Ok))
                std::swap_ranges(_asr(byte*, &a.value), _asr(byte*, &a.value) + sizeof(ResultStorageType<T>), _asr(byte*, &b.value));
            swap(a.status, b.status);
        }

        friend struct sys::ResultCommon<Result, T, void>;
    private:
        union
        {
            byte none;
            ResultStorageType<T> value;
        };
        ResultStatus status = ResultStatus::Empty;
    };

    /// @brief Awaiter to enable short-circuiting, à la rustlang `operator?`.
    /// @tparam T The type of the value to hold.
    /// @tparam Err The type of the error to hold.
    template <typename T, typename Err>
    struct ResultAwaiter
    {
        _inline_always constexpr bool await_ready() const noexcept
        {
            return res;
        }
        template <typename Promise>
        _inline_always constexpr void await_suspend(std::coroutine_handle<Promise> parent)
        {
            if constexpr (!std::is_same<Err, void>::value)
                parent.promise().return_value(res.takeError());
            else
                []<bool _false = false>
                {
                    static_assert(_false, "`Result<...>::operator?` requires `typename Err` to be returnable in the current scope!");
                }();

            if constexpr (requires { parent.promise().continuation.resume(); })
                parent.promise().continuation.resume();
        }
        _inline_always constexpr T await_resume() const noexcept(std::is_same<T, void>::value)
        {
            return res.takeValue();
        }

        friend struct sys::Result<T, Err>;
    private:
        Result<T, Err>& res;
    };
} // namespace sys
