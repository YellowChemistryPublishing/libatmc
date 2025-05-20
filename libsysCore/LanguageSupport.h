#pragma once

#include <cstdint>
#include <print>
#include <source_location>
#include <stdfloat>
#include <type_traits>
#include <typeinfo>

#include <CompilerWarnings.h>

using byte = unsigned char;
using sbyte = signed char;
using ushort = unsigned short;
using uint = unsigned int;
using ulong = unsigned long;
using llong = long long;
using ullong = unsigned long long;

using i8 = int_least8_t;
using i16 = int_least16_t;
using i32 = int_least32_t;
using i64 = int_least64_t;

using u8 = uint_least8_t;
using u16 = uint_least16_t;
using u32 = uint_least32_t;
using u64 = uint_least64_t;

using sz = size_t;
using ssz = ptrdiff_t;

#if !__STDCPP_FLOAT32_T__
using f32 = std::float32_t;
#endif
#if !__STDCPP_FLOAT64_T__
using f64 = std::float64_t;
#endif

struct unsafe_tag
{ };
constexpr unsafe_tag unsafe {};

/// @def _inline_always
/// @brief Force inline a function.
#define _inline_always [[gnu::always_inline]] inline
/// @def _inline_never
/// @brief Force noinline a function.
#define _inline_never [[gnu::noinline]]
/// @def _pure
/// @brief Mark a function as pure.
#define _pure [[gnu::pure]]
/// @def _const
/// @brief Mark a function as const.
#define _const [[gnu::const]]
/// @def _restrict
/// @brief Mark a parameter (or this) as restrict.
#define _restrict __restrict__

#define _as(T, expr) static_cast<T>(expr)
#define _asd(T, expr) dynamic_cast<T>(expr)
#define _asc(T, expr) const_cast<T>(expr)
#define _asr(T, expr) reinterpret_cast<T>(expr)
#define _asi(T, expr) ::sys::numeric_cast<T>(expr, unsafe)

/// @def _throw(value)
/// @brief Logs a source location, and throws the value of the expression `value`.
#define _throw(value)                                                                                                                                                          \
    do                                                                                                                                                                          \
    {                                                                                                                                                                           \
        std::source_location __srcLoc = std::source_location::current();                                                                                                        \
        _push_nowarn(_clWarn_use_after_free);                                                                                                                                 \
        std::println(stderr, "In function `{}` at \"{}:{}:{}\" - Throwing `{}`.", __srcLoc.function_name(), __srcLoc.file_name(), int(__srcLoc.line()), int(__srcLoc.column()), \
                     typeid(decltype(value)).name());                                                                                                                           \
        _pop_nowarn();                                                                                                                                                         \
        throw(value);                                                                                                                                                           \
    }                                                                                                                                                                           \
    while (false)

/// @def _fence_value_return(val, retCond)
/// @brief Return `val` if `retCond` is true.
#define _fence_value_return(val, retCond) \
    if (retCond)                           \
        return val;
/// @def _fence_value_co_return(val, retCond)
/// @brief Coroutine-return `val` if `retCond` is true.
#define _fence_value_co_return(val, retCond) \
    if (retCond)                              \
        co_return val;
/// @def _fence_contract_enforce(cond)
/// @brief Enforce a contract, throwing a `ContractViolationException` if `cond` is false.
#define _fence_contract_enforce(cond)                                                                                    \
    do                                                                                                                    \
    {                                                                                                                     \
        const bool __expr = cond;                                                                                         \
        if (!__expr)                                                                                                      \
            _throw(::sys::ContractViolationException("Contract violated, condition `" #cond "` evaluated to `false`.")); \
        [[assume(__expr)]];                                                                                               \
    }                                                                                                                     \
    while (false)

namespace sys
{
    template <typename T>
    concept INumber = std::integral<T> || std::floating_point<T>;

    template <typename T, typename U = void>
    concept IEnumerable = std::is_array_v<T> || requires(T range, std::remove_cvref_t<decltype(range.begin())> it) {
        range.begin();
        range.end();

        range.begin() != range.end();
        ++it;

        requires ((std::same_as<U, void> && requires { *range.begin(); }) || std::convertible_to<decltype(*range.begin()), std::add_lvalue_reference_t<std::add_const_t<U>>>);
    } || requires(T range, std::remove_cvref_t<decltype(begin(range))> it) {
        begin(range);
        end(range);

        begin(range) != end(range);
        ++it;

        requires ((std::same_as<U, void> && requires { *begin(range); }) || std::convertible_to<decltype(*begin(range)), std::add_lvalue_reference_t<std::add_const_t<U>>>);
    };
} // namespace sys
