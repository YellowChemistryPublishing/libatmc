/* module; */
#pragma once

#include <cmath>
#include <cstdint>
#include <cxxutil.h>
#include <initializer_list>

/* export module core.Math; */

/* export */ namespace atmc::math
{
	template <typename VectorData, size_t N, typename T = float>
	struct VectorOperators
	{
		using ValueType = T;

		constexpr VectorOperators() noexcept = default;
		
		constexpr T& operator[](size_t i) noexcept
		{
			return static_cast<VectorData*>(this)->elements[i];
		}
		constexpr T operator[](size_t i) const noexcept
		{
			return static_cast<const VectorData*>(this)->elements[i];
		}
		consteval static size_t size() noexcept
		{
			return N;
		}

		constexpr friend bool operator==(const VectorData& lhs, const VectorData& rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				if (lhs[i] != rhs[i])
					return false;
			return true;
		}

		constexpr friend VectorData operator+(const VectorData& lhs, const VectorData& rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] += rhs[i];
			return ret;
		}
		constexpr friend VectorData operator-(const VectorData& lhs, const VectorData& rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] -= rhs[i];
			return ret;
		}
		constexpr friend VectorData operator*(const VectorData& lhs, const VectorData& rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] *= rhs[i];
			return ret;
		}
		constexpr friend VectorData operator*(const VectorData& lhs, T rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] *= rhs;
			return ret;	
		}
		constexpr friend VectorData operator*(T lhs, const VectorData& rhs) noexcept
		{
			VectorData ret = rhs;
			for (size_t i = 0; i < N; i++)
				ret[i] *= lhs;
			return ret;
		}
		constexpr friend VectorData operator/(const VectorData& lhs, const VectorData& rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] /= rhs[i];
			return ret;
		}
		constexpr friend VectorData operator/(const VectorData& lhs, T rhs) noexcept
		{
			VectorData ret = lhs;
			for (size_t i = 0; i < N; i++)
				ret[i] /= rhs;
			return ret;
		}

		constexpr VectorData& operator+=(const VectorData& rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] += rhs[i];
			return *static_cast<VectorData*>(this);
		}
		constexpr VectorData& operator-=(const VectorData& rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] -= rhs[i];
			return *static_cast<VectorData*>(this);
		}
		constexpr VectorData& operator*=(const VectorData& rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] *= rhs[i];
			return *static_cast<VectorData*>(this);
		}
		constexpr VectorData& operator*=(T rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] *= rhs;
			return *static_cast<VectorData*>(this);
		}
		constexpr VectorData& operator/=(const VectorData& rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] /= rhs[i];
			return *static_cast<VectorData*>(this);
		}
		constexpr VectorData& operator/=(T rhs) noexcept
		{
			for (size_t i = 0; i < N; i++)
				static_cast<VectorData*>(this)->elements[i] /= rhs;
			return *static_cast<VectorData*>(this);
		}

		constexpr T magnitude2() const noexcept
		{
			T ret = 0;
			for (size_t i = 0; i < N; i++)
				ret += static_cast<VectorData*>(this)->elements[i] * static_cast<VectorData*>(this)->elements[i];
			return ret;
		}
		constexpr T magnitude() const noexcept
		{
			if constexpr (std::is_same<T, float>::value)
				return std::sqrtf(static_cast<VectorData*>(this)->magnitude2());
			else if constexpr (std::is_same<T, double>::value)
				return std::sqrt(static_cast<VectorData*>(this)->magnitude2());
			else if constexpr (std::is_same<T, long double>::value)
				return std::sqrtl(static_cast<VectorData*>(this)->magnitude2());
			else return T(std::sqrt(static_cast<VectorData*>(this)->magnitude2()));
		}
	private:
		T elements[N] { };
	};

	template <typename T = float>
	struct Vector2T : public VectorOperators<Vector2T<T>, 2, T>
	{
		__push_nowarn(__clWarn_pedantic);
		union
		{
			T elements[2];
			struct
			{
				T x, y;
			};
		};
		__pop_nowarn();

		using VectorOperators<Vector2T<T>, 2, T>::VectorOperators;

		constexpr Vector2T(T x, T y) noexcept : x(x), y(y)
		{ }
	};
	template <typename T = float>
	struct Vector3T : public VectorOperators<Vector3T<T>, 3, T>
	{
		__push_nowarn(__clWarn_pedantic);
		union
		{
			T elements[3];
			struct
			{
				T x, y, z;
			};
		};
		__pop_nowarn();

		using VectorOperators<Vector3T<T>, 3, T>::VectorOperators;

		constexpr Vector3T(T x, T y, T z) noexcept : x(x), y(y), z(z)
		{ }
	};
	template <typename T = float>
	struct Vector4T : public VectorOperators<Vector4T<T>, 4, T>
	{
		__push_nowarn(__clWarn_pedantic);
		union
		{
			T elements[4];
			struct
			{
				T x, y, z, w;
			};
		};
		__pop_nowarn();

		using VectorOperators<Vector4T<T>, 4, T>::VectorOperators;

		constexpr Vector4T(T x, T y, T z, T w) noexcept : x(x), y(y), z(z), w(w)
		{ }
	};
	template <size_t N, typename T = float>
	requires (N > 4)
	struct VectorT : public VectorOperators<VectorT<N, T>, N, T>
	{
		__push_nowarn(__clWarn_pedantic);
		union
		{
			T elements[N];
			struct
			{
				T x, y, z, w;
			};
		};
		__pop_nowarn();

		using VectorOperators<VectorT<N, T>, N, T>::VectorOperators;
		
		constexpr VectorT(std::initializer_list<T> elements) noexcept : elements { elements }
		{ }
	};
	template <typename T>
	concept VectorType = std::same_as<T, Vector2T<typename T::ValueType>> || std::same_as<T, Vector3T<typename T::ValueType>> || std::same_as<T, Vector4T<typename T::ValueType>> || std::same_as<T, VectorT<T::size(), typename T::ValueType>>;

	template <VectorType T>
	T dot(const T& lhs, const T& rhs) noexcept
	{
		T ret = 0;
		for (size_t i = 0; i < T::size(); i++)
			ret += lhs[i] * rhs[i];
		return ret;
	}
	template <typename T = float>
	Vector3T<T> cross(const Vector3T<T>& lhs, const Vector3T<T>& rhs) noexcept
	{
		return Vector3T<T>
		{
			lhs.y() * rhs.z() - lhs.z() * rhs.y(),
			lhs.z() * rhs.x() - lhs.x() * rhs.z(),
			lhs.x() * rhs.y() - lhs.y() * rhs.x()
		};
	}

	using Vector3Int8 = Vector3T<int8_t>;
	using Vector3Int16 = Vector3T<int16_t>;
	using Vector3 = Vector3T<>;

	struct Matrix3x3UInt8
	{
		uint8_t m00, m01, m02;
		uint8_t m10, m11, m12;
		uint8_t m20, m21, m22;

		constexpr bool operator==(const Matrix3x3UInt8& rhs) const noexcept = default;
	};
}
