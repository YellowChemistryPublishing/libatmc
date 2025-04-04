/* module; */
#pragma once

#include <cmath>
#include <cstring>
#include <cxxutil.h>
#include <cxxutil.hpp>
#include <exception>
#include <print>

#include <CompilerWarnings.h>
#include <Exception.hpp>

/* export module core.Library:Result; */

/* import core.Fundamental; */

/* export */ namespace atmc
{
	/// @brief A result type that can hold either a value or an error.
	/// @tparam T The type of the value to hold.
	/// @tparam Err The type of the error to hold.
	template <typename T, typename Err = void>
	struct Result
	{
		/// @brief Constructs a result with a value.
		/// @param value Value to move into the result.
		inline Result(T&& value) : value(std::forward<T>(value)), ok(1)
		{ }
		/// @brief Constructs a result with an error.
		/// @param error Error to move into the result.
		inline Result(Err&& error) : error(std::forward<Err>(error)), ok(0)
		{ }
		/// @brief Move constructor.
		/// @param other Result to move from.
		inline Result(Result&& other) : ok(other.ok)
		{
			other.ok = -1;

			__push_nowarn(__clWarn_bad_offsetof);
			static_assert(offsetof(Result, value) == offsetof(Result, error), "Result value/error union must start at the same byte.");
			__pop_nowarn();

			__push_nowarn(__clWarn_nontrivial_memcpy);
			std::memcpy(&this->value, &other.value, std::max(sizeof(T), sizeof(Err)));
			__pop_nowarn();
		}
		inline ~Result() noexcept(false)
		{
			if (this->ok == 1)
				this->value.~T();
			else if (this->ok == 0)
			{
				this->error.~Err();
				__fence_contract_enforce(false && "Result with bad value ignored!");
			}
		}

		/// @brief Whether the result is good.
		inline operator bool ()
		{
			return this->ok == 1;
		}

		/// @brief Takes the value if the result is good.
		/// @return The value held by the result.
		inline T takeValue()
		{
			__fence_contract_enforce(this->ok == 1 && "Taking value for a bad or empty result!");
			this->ok = -1;
			return std::move(this->value);
		}
		/// @brief Takes the value if the result is good, otherwise returns a default value, executing an action with the held error.
		/// @tparam ...Args Parameter pack to construct the default value with.
		/// @tparam Func Type of the action to execute with the error.
		/// @param ...args Arguments to construct the default value with.
		/// @param withError Callback to execute with the error.
		/// @return The value held by the result, or a default value.
        /// @attention This crashes the compiler.
		//	template <typename... Args, typename Func = decltype([](Err) { })>
		//	requires std::same_as<decltype(std::declval<Func>()(std::move(std::declval<Err>()))), void> && std::invocable<Func, Err&&>
		//	inline T valueOr(Args&&... args, Func&& withError = Func())
		//	{
		//		if (this->ok == 1) [[likely]]
		//			return this->takeValue();
		//		else
		//		{
		//			if (this->ok == 0) [[likely]]
		//				withError(std::move(this->takeError()));
		//			return T(std::forward<Args>(args)...);
		//		}
		//	}
		/// @brief Takes the value if the result is good, otherwise throws the error.
		/// @return The value held by the result.
		/// @attention You are banned from using this function in production code.
		inline T valueOrThrow()
		{
			if (this->ok == 1) [[likely]]
				return this->takeValue();
			else if (this->ok == 0)
				__throw(std::move(this->takeError()));
			else __throw(nullptr);
		}
		/// @brief Takes the error if the result is bad.
		/// @return The error held by the result.
		inline Err takeError()
		{
			__fence_contract_enforce(this->ok == 0 && "Taking error for a good or empty result!");
			this->ok = -1;
			return std::move(this->error);
		}
	private:
		union
		{
			T value;
			Err error;
		};
		char ok;
	};

	/// @brief A result type that can hold either a value or nothing.
	/// @tparam T The type of the value to hold.
	template <typename T>
	struct Result<T, void>
	{
		/// @brief Constructs a result with a value.
		/// @param value 
		inline Result(T&& value) : ok(1)
		{
			new (&this->value) T(std::forward<T>(value));
		}
		/// @brief Constructs a result with nothing.
		inline Result(std::nullptr_t) : ok(0)
		{ }
		inline ~Result()
		{
			if (this->ok == 1)
				reinterpret_cast<T*>(&this->value)->~T();
		}

		/// @brief Whether the result is good.
		inline operator bool ()
		{
			return this->ok == 1;
		}

		/// @brief Takes the value if the result is good.
		/// @return The value held by the result.
		inline T takeValue()
		{
			__fence_contract_enforce(this->ok == 1 && "Taking value for a bad result!");
			this->ok = -1;
			return std::move(this->value);
		}
	private:
		alignas(T) unsigned char value[sizeof(T)];
		char ok;
	};
}
