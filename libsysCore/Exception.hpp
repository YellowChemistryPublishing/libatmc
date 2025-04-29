#pragma once

#include <cxxabi.h>
#include <memory>
#include <utility>

namespace atmc
{
	/// @brief Base type for all managed exceptions.
	/// @note
	/// Inherits from `std::exception`, no need to catch this type specifically.
	/// Pass `byref`.
	struct ManagedException : public std::exception
	{
		/// @brief What went wrong.
		/// @return The reason for the exception.
		constexpr virtual const char* what() const noexcept override = 0;
	};
	/// @brief Thrown when an `FencedPointer<T>` is dereferenced, addition-assigned, subtraction-assigned, incremented, or decremented when `nullptr`.
	/// @note Pass `byref`.
	struct NullPointerException : public ManagedException
	{
		/// @brief What went wrong.
		/// @return Literal string.
		constexpr const char* what() const noexcept override
		{
			return "Attempted to access, increment, or decrement a null pointer.";
		}
	};
	
	/// @brief A pointer that throws on attempted `nullptr` dereference.
	/// @tparam T The type of the pointer.
	/// @note Pass `byval`.
	template <typename T>
	struct FencedPointer
	{
		/// @brief Construct `FencedPointer<T>` with the value `nullptr`.
		inline FencedPointer() noexcept = default;
		/// @brief Construct `FencedPointer<T>` with a value.
		/// @param value The value to assign to the pointer.
		inline FencedPointer(T* value) noexcept : value(value)
		{ }
		/// @brief Copy constructor.
		/// @param other The other `FencedPointer<T>` to copy.
		inline FencedPointer(const FencedPointer& other) noexcept : value(other.value)
		{ }

		/// @brief Assignment operator.
		/// @param value The value to assign to the pointer.
		/// @return Reference to `*this`.
		inline FencedPointer& operator=(T* value) noexcept
		{
			this->value = value;
			return *this;
		}
		/// @brief Copy assignment operator.
		/// @param other The other `FencedPointer<T>` to copy.
		/// @return Reference to `*this`.
		inline FencedPointer& operator=(const FencedPointer& other) noexcept
		{
			this->value = other.value;
			return *this;
		}

		/// @brief Obtain the pointer value, without fencing it against `nullptr`.
		/// @return The underlying pointer value.
		/// @note This function is marked unchecked because it will not throw if `this->value == nullptr`.
		inline T* unchecked() const noexcept
		{
			return this->value;
		}
	 	/// @brief Obtain the pointer value, throwing if `nullptr`.
	 	/// @return The underlying pointer value.
	 	[[nodiscard]] inline T& operator*() const
		{
			return *this->obtainOrThrow();
		}
	 	/// @brief Obtain the pointer value, throwing if `nullptr`.
	 	/// @return The underlying pointer value.
	 	[[nodiscard]] inline T* operator->() const
		{
			return this->obtainOrThrow();
		}
		/// @brief Obtain value at index, throwing if `nullptr`.
		/// @param index The index to access.
		/// @return The value at the index.
		[[nodiscard]] inline T& operator[](std::size_t index) const
		{
			return this->obtainOrThrow()[index];
		}
		
		/// @brief Spaceship operator.
		/// @param other The other `FencedPointer<T>` to compare against.
		/// @return The comparison result.
		inline auto operator<=>(const FencedPointer& other) const noexcept = default;

		/// @brief Prefix increment operator.
		/// @return Reference to `*this`.
		inline FencedPointer& operator++() noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			++this->value;
			return *this;
		}
		/// @brief Prefix decrement operator.
		/// @return Reference to `*this`.
		inline FencedPointer& operator--() noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			--this->value;
			return *this;
		}
		/// @brief Postfix increment operator.
		/// @return The value before incrementing.
		inline FencedPointer operator++(int) noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			FencedPointer ret = *this;
			++this->value;
			return ret;
		}
		/// @brief Postfix decrement operator. 
		/// @return The value before decrementing.
		inline FencedPointer operator--(int) noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			FencedPointer ret = *this;
			--this->value;
			return ret;
		}
		/// @brief Addition operator.
		/// @param offset The offset to add.
		/// @return The pointer with the offset added.
		inline FencedPointer operator+(ptrdiff_t offset) const noexcept
		{
			return FencedPointer { this->value + offset };
		}
		/// @brief Subtraction operator.
		/// @param offset The offset to subtract.
		/// @return The pointer with the offset subtracted.
		inline FencedPointer operator-(ptrdiff_t offset) const noexcept
		{
			return FencedPointer { this->value - offset };
		}
		/// @brief Addition operator.
		/// @param offset The offset to add.
		/// @return The pointer with the offset added.
		inline ptrdiff_t operator-(const FencedPointer& other) const noexcept
		{
			return this->value - other.value;
		}
		/// @brief Addition assignment operator.
		/// @param offset The offset to add.
		/// @return Reference to `*this`.
		inline FencedPointer& operator+=(ptrdiff_t offset) noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			this->value += offset;
			return *this;
		}
		/// @brief Subtraction assignment operator.
		/// @param offset The offset to subtract.
		/// @return Reference to `*this`.
		inline FencedPointer& operator-=(ptrdiff_t offset) noexcept
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			this->value -= offset;
			return *this;
		}
	private:
		T* value = nullptr;

		/// @brief Obtain the pointer value, throwing if `nullptr`.
		/// @return The underlying pointer value.
		[[nodiscard]] inline T* obtainOrThrow() const
		{
			__push_nowarn(__clWarn_pedantic);
			void* jmp[2] { &&ThrowNull, &&Op };
			goto *jmp[__sc(bool, this->value)];
			__pop_nowarn();
		ThrowNull:
			__throw(NullPointerException());
		Op:
			return this->value;
		}
	};

	#if !defined(__clang__) && defined(__GNUC__)
	inline std::unique_ptr<char, void (*)(void*)> exceptionTypeName(std::exception_ptr ex)
	{
		return std::unique_ptr<char, void (*)(void*)>(abi::__cxa_demangle(ex.__cxa_exception_type()->name() /* Contract implied: `name != nullptr`. */, NULL, NULL, NULL), std::free);
	}
	#else
	inline std::unique_ptr<const char, void (*)(const void*)> exceptionTypeName([[maybe_unused]] std::exception_ptr ex)
	{
		return std::unique_ptr<const char, void (*)(const void*)>("<unknown>", [](const void*) { });
	}
	#endif

	/// @brief Terminate the program.
	/// @note Pass `byref`.
	struct TerminateException : public ManagedException
	{
		/// @brief On construction, terminate the program.
		inline TerminateException() noexcept
		{
			std::terminate();
		}

		/// @brief Never called.
		/// @return [[noreturn]]
		[[noreturn]] constexpr const char* what() const noexcept override
		{
			std::unreachable();
		}
	};
	/// @brief Thrown when a contract is violated.
	/// @note
	/// Please do not throw this exception directly, use the `__fence_contract_enforce` macro instead. Don't catch this exception either.
	/// Pass `byref`.
	struct ContractViolationException : public ManagedException
	{
		/// @brief Construct the exception with a reason, which is logged on construction.
		/// @param why The reason the contract was violated.
		template <size_t N>
		inline ContractViolationException(const char (&why)[N]) noexcept : why(why)
		{
			std::println(stderr, "Note: {}", why);
		}

		/// @brief What went wrong.
		/// @return The reason a contract was violated.
		inline const char* what() const noexcept override
		{
			return this->why;
		}
	private:
		const char* why;
	};
}