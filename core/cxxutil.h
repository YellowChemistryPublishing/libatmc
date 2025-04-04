#pragma once

#include <print>
#include <source_location>
#include <typeinfo>

#include <CompilerWarnings.h>

/// @def __throw(value)
/// @brief Logs a source location, and throws the value of the expression `value`.
#define __throw(value) do \
{ \
	std::source_location __srcLoc = std::source_location::current(); \
	__push_nowarn(__clWarn_use_after_free); \
	std::println \
	( \
		stderr, "In function `{}` at \"{}:{}:{}\" - Throwing `{}`.", \
		__srcLoc.function_name(), __srcLoc.file_name(), (int)__srcLoc.line(), (int)__srcLoc.column(), \
		typeid(decltype(value)).name() \
	); \
	__pop_nowarn(); \
	throw (value); \
} \
while (false)

/// @def __fence_value_return(val, retCond)
/// @brief Return `val` if `retCond` is true.
#define __fence_value_return(val, retCond) if (retCond) return val;
/// @def __fence_value_co_return(val, retCond)
/// @brief Coroutine-return `val` if `retCond` is true.
#define __fence_value_co_return(val, retCond) if (retCond) co_return val;
/// @def __fence_contract_enforce(cond)
/// @brief Enforce a contract, throwing a `ContractViolationException` if `cond` is false.
#define __fence_contract_enforce(cond) do { const bool __expr = cond; if (!__expr) __throw(ContractViolationException("Contract violated, condition `" #cond "` evaluated to `false`.")); [[assume(__expr)]]; } while (false);

#define __async(void) ::atmc::Async
