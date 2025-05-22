# Style Guide

## Eliminating Error Prone Practice

CMake settings have configured compilation under gcc to use `-Wall -Wextra -Wpedantic -Werror`. To suppress false positives, see the useful macros in the `CompilerWarnings.h` header.

You may not `throw` anything for the purpose of error checking.
An exception is granted for fatal program conditions, such as that of a contract violation.
In the case that a throw expression is required, you are forbidden from using the `throw` keyword directly, please see `_throw(expr)`.
Correctly written production code is banned from executing a `_throw(expr)` expression, except for, once again, fatal error handling.

You _should_ not `catch(...)` anything, with the possible exception of hardware/system related exceptions, for example  `std::bad_alloc`, for producing debugging information.

Due to partial requirements to support exceptions, your code must expect that control flow may return to the caller at any time.
Hence, you must make sure that your code _never_ abandons unmanaged resources, i.e. (wrong) `spinLock.lock(); ... /* _throw(expr) */ ... spinLock.unlock();`.
The natural corollary of this is that all synchronisation mechanisms (i.e. `sys::SpinLock`) must never have `.lock()` or `.unlock()` invoked directly--
    you should use some lock guard instead.

No C-style casts! Shortened macros for all casts are provided for convenience, please use those! (i.e. `_as(T, expr)` equiv. `static_cast<T>(expr)`.)
Alternatively, for many builtin types, prefer invoking their constructors explicitly. (i.e. `int(2.0f)`.)

Please don't let constructors silently fail!
Instead, include the static member function `static Result<T> ctor(...)`, and include in the body of the analogous constructor `_assert_ctor_can_fail()`.
The convenience macro `_ct(T, ...)` will search for this `ctor` function first, and then resort to invoking the constructor.

## Informed Annotations

Documenting functions using the doxygen syntax in comments is highly recommended for production code.
(Non-Enum) Types must be additionally annotated with ``/// @note Pass `byval`.``, ``/// @note Pass `byref`.``, or ``/// @note Pass `byptr`.`` indicating whether a type is best passed as `T`, `(const) T&`, or `(const) T (const)*`.
Any `FencedPointer<T>` accesses that look like `&*ptr` must be accompanied by a comment on the right of the expression ``/* Contract implied: `ptr != nullptr`. */``.
Types that act as static classes must be annotated with `/// @note Static class.`.
The only mandatory field for production code is an `@attention Lifetime assumptions!` clause on functions, with a `cpp` block highlighting the lifetime requirements of non-trivial parameters.

Any error checking passed up to a caller must be administered with short-circuiting "fences".
`_fence_value_(co_)return(val, cond)` will return `val` iff. `cond` evaluates to `true`.
`_fence_result_(co_)return(rValRef, out)` will use in assignment the `Result<...>` object from `rValRef`, returning its error via `Result<...>::takeError()` in the error case,
    or otherwise, moving its result value into `out` via `Result<...>::takeValue()`.
When inside a coroutine, `co_await result` is semantically identical to `_fence_result_co_return(rValRef, out)`.

Runtime domain/input validation may be achieved through contracts. Since C++ has no standard implemented contracts yet, you must use `_fence_contract_enforce(cond)`.
Note that contract conditions are passed to `const bool __expr = cond; [[assume(__expr)]];`, so you must ensure those statements will not be ill-formed.

Parameters of type array-as-pointer must be passed by the array notation `... func(T arr[])`, rather than `... func(T* arr)`.

Where applicable and valid, `_restrict` must apply to pointers.

Overriden virtual member functions must be annotated with the `override` specifier.

All functions, where correct to, must be annotated with `_const` (`[[gnu::const]]`), or, where otherwise correct to, `_pure` (`[[gnu::pure]]`).
Don't forget `[[nodiscard]]` too!

Where template parameters have restricted domain or constraints, they must be specified with a `requires` clause.

## Library Checklist

If you check all these boxes, you're code is probably sufficiently well thought out.

 - Functions and variables marked with `noexcept`, `const`, `_pure`, `_const`, `_restrict`, `[[nodiscard]]` where applicable.
 - `constexpr` anything that moves.
 - You are using `Result<...>` for error handling.
 - An exception thrown at any point in my code would not cause a memory leak (or, it is sufficiently justified that one must not occur).
 - For custom types, a function named `swap` is implemented in the same scope, and used in the type's move constructor.
    - (Please do the `friend constexpr void swap(T& a, T& b) { using std::swap; ... }` trick please.)
 - When applicable, a type has a member function with declaration `sz hashCode()` is provided.
