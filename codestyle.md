# Style Guide

## Eliminating Error Prone Practice

You may not `throw` anything for the purpose of error checking.
An exception is granted for fatal program conditions, such as that of a contract violation.
In the case that a throw expression is required, you are forbidden from using the `throw` keyword directly, please see `__throw(expr)`.
Correctly written production code is banned from executing a `__throw(expr)` expression, except for, once again, fatal error handling.

You _should_ not `catch(...)` anything, with the possible exception of hardware/system related exceptions, for example  `std::bad_alloc`, for producing debugging information.

Due to partial requirements to support exceptions, your code must expect that control flow may return to the caller at any time.
Hence, you must make sure that your code _never_ abandons unmanaged resources, i.e. (wrong) `spinLock.lock(); ... /* __throw(expr) */ ... spinLock.unlock();`.
The natural corollary of this is that all synchronisation mechanisms (i.e. `atmc::SpinLock`) must never have `.lock()` or `.unlock()` invoked directly--
    you should use some lock guard instead.

CMake settings have configured compilation under gcc to use `-Wall -Wextra -Wpedantic -Werror`. To suppress false positives, see the useful macros in the `CompilerWarnings.h` header.

## Informed Annotations

Documenting functions using the doxygen syntax in comments is highly recommended for production code.
(Non-Enum) Types must be additionally annotated with ``/// @note Pass `byval`.``, ``/// @note Pass `byref`.``, or ``/// @note Pass `byptr`.`` indicating whether a type is best passed as `T`, `(const) T&`, or `(const) T (const)*`.
Any `FencedPointer<T>` accesses that look like `&*ptr` must be accompanied by a comment on the right of the expression ``/* Contract implied: `ptr != nullptr`. */``.
Types that act as static classes must be annotated with `/// @note Static class.`.
The only mandatory field for production code is an `@attention Lifetime assumptions!` clause on functions, with a `cpp` block highlighting the lifetime requirements of non-trivial parameters.

No C-style casts! Shortened macros for all casts are provided for convenience, please use those! (i.e. `__sc(T, expr)` equiv. `static_cast<T>(expr)`.)

Where template parameters have restricted domain or constraints, they must be specified with a `requires` clause.

Where applicable and valid, `__restrict__` must apply to pointers.
All functions, where correct to, must be annotated with `[[gnu::const]]`, or, where otherwise correct to, `[[gnu::pure]]`.

Runtime domain/input validation may be achieved through contracts. Since C++ has no standard implemented contracts yet, you must use `__fence_contract_enforce(cond)`.
Note that contract conditions are passed to `const bool __expr = cond; [[assume(__expr)]];`, so you must ensure those statements will not be ill-formed.

Any error checking passed up to a caller must be administered with short-circuiting "fences".
`__fence_value_(co_)return(val, cond)` will return `val` iff. `cond` evaluates to `true`.
`__fence_result_(co_)return(rValRef, out)` will use in assignment the `Result<...>` object from `rValRef`, returning its error via `Result<...>::takeError()` in the error case,
    or otherwise, moving its result value into `out` via `Result<...>::takeValue()`.

Parameters of type array-as-pointer must be passed by the array notation `... func(T arr[])`, rather than `... func(T* arr)`.

Overriden virtual member functions must be annotated with the `override` specifier.
