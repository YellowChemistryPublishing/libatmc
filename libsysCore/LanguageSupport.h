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

/// @def __inline_always
/// @brief Force inline a function.
#define _inline_always [[gnu::always_inline]] inline
/// @def __inline_never
/// @brief Force noinline a function.
#define _inline_never [[gnu::noinline]]
/// @def __pure
/// @brief Mark a function as pure.
#define _pure [[gnu::pure]]
/// @def __const
/// @brief Mark a function as const.
#define _const [[gnu::const]]
/// @def __restrict
/// @brief Mark a parameter (or this) as restrict.
#define _restrict __restrict__
/// @def __counted_by
/// @brief Mark a parameter (or this) as restrict.
#define _counted_by(var) [[gnu::counted_by(var)]]

#define _as(T, expr) static_cast<T>(expr)
#define _asd(T, expr) dynamic_cast<T>(expr)
#define _asc(T, expr) const_cast<T>(expr)
#define _asr(T, expr) reinterpret_cast<T>(expr)

/// @def __throw(value)
/// @brief Logs a source location, and throws the value of the expression `value`.
#define __throw(value)                                                                                                                                                          \
    do                                                                                                                                                                          \
    {                                                                                                                                                                           \
        std::source_location __srcLoc = std::source_location::current();                                                                                                        \
        __push_nowarn(__clWarn_use_after_free);                                                                                                                                 \
        std::println(stderr, "In function `{}` at \"{}:{}:{}\" - Throwing `{}`.", __srcLoc.function_name(), __srcLoc.file_name(), int(__srcLoc.line()), int(__srcLoc.column()), \
                     typeid(decltype(value)).name());                                                                                                                           \
        __pop_nowarn();                                                                                                                                                         \
        throw(value);                                                                                                                                                           \
    }                                                                                                                                                                           \
    while (false)

/// @def __fence_value_return(val, retCond)
/// @brief Return `val` if `retCond` is true.
#define __fence_value_return(val, retCond) \
    if (retCond)                           \
        return val;
/// @def __fence_value_co_return(val, retCond)
/// @brief Coroutine-return `val` if `retCond` is true.
#define __fence_value_co_return(val, retCond) \
    if (retCond)                              \
        co_return val;
/// @def __fence_contract_enforce(cond)
/// @brief Enforce a contract, throwing a `ContractViolationException` if `cond` is false.
#define __fence_contract_enforce(cond)                                                                                    \
    do                                                                                                                    \
    {                                                                                                                     \
        const bool __expr = cond;                                                                                         \
        if (!__expr)                                                                                                      \
            __throw(::sys::ContractViolationException("Contract violated, condition `" #cond "` evaluated to `false`.")); \
        [[assume(__expr)]];                                                                                               \
    }                                                                                                                     \
    while (false)
