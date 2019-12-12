#ifndef FP_MATH_DY_H
#define FP_MATH_DY_H

#include <stdint.h>

#ifdef __cplusplus
namespace fp64 {
	extern "C" {
		namespace fpc {
#endif // __cplusplus

			typedef struct Fp64 {
				int64_t internal;
			} Fp64;

			// Ops
			extern Fp64 fp64_add(Fp64 a, Fp64 b);
			extern Fp64 fp64_sub(Fp64 a, Fp64 b);
			extern Fp64 fp64_mul(Fp64 a, Fp64 b);
			extern Fp64 fp64_div(Fp64 a, Fp64 b);
			extern Fp64 fp64_mod(Fp64 a, Fp64 b);

			// Maths
			extern Fp64 fp64_pow(Fp64 f, uint32_t exponent);
			extern Fp64 fp64_sqrt(Fp64 f, Fp64 precision);
			extern Fp64 fp64_easy_sqrt(Fp64 f);
			extern Fp64 fp64_reciprocal_sqrt(Fp64 f, uint32_t iterations);
			extern Fp64 fp64_sin(Fp64 f);
			extern Fp64 fp64_cos(Fp64 f);
			extern Fp64 fp64_half(Fp64 f);
			extern Fp64 fp64_double(Fp64 f);

			// Int conversions
			extern Fp64 fp64_from_i32(int32_t i);
			extern Fp64 fp64_from_i64(int64_t i);
			extern int32_t fp64_to_i32(Fp64 f);
			extern int64_t fp64_to_i64(Fp64 f);

			// Float conversions
			extern Fp64 fp64_from_f32(float i);
			extern Fp64 fp64_from_f64(double i);
			extern float fp64_to_f32(Fp64 f);
			extern double fp64_to_f64(Fp64 f);

#ifdef __cplusplus
		} // namespace fpc
	} // extern C

	/**
	 * Class wrapped around the equivalent C struct and its associated
	 * functions.
	 *
	 * Instead of constructors,there are several static methods of the
	 * form Fp64::FromType(value) that can be used to create the fixed
	 * point value from a primitive type.
	 *
	 * Operators are overloaded for ease of use, including explicit
	 * conversion operators allowing Fp64 objects to be casted back to
	 * primitives numeric types.
	 *
	 * A few functions offer access to the internal representation,
	 * it is ill-advised to make use of these for any sort of
	 * arithmetic/manipulation, as the scale of the internal integer
	 * may change in future versions.
	 */

	class Fp64 {
	private:
		fpc::Fp64 m_Internal;
	public:
		// 'Constructors':
		inline Fp64(fpc::Fp64 Internal) : m_Internal(Internal) {};

		inline Fp64() {
			m_Internal = fpc::Fp64{ 0 };
		}

		inline Fp64(float f) {
			*this = FromFloat(f);
		}

		inline Fp64(int i) {
			*this = FromInt32(i);
		}

		inline Fp64(double d) {
			*this = FromDouble(d);
		}

		/**
		 * Creates an Fp64 with the specified internal
		 * 64-bit int representation. This should be used
		 * with caution, as the scale is not guaranteed to
		 * stay the same in future versions!
		 */
		inline static Fp64 WithInternalRepresentation(int64_t i)
		{
			return Fp64({ fpc::fp64_from_i64(i) });
		}

		/**
		 * Creates an Fp64 that matches the provided float
		 * value closely. Casting the resulting Fp64 back
		 * to a float carries no guarantees that the value
		 * will be identical to when it was initially provided.
		 */
		inline static Fp64 FromFloat(float f)
		{
			return Fp64(fpc::fp64_from_f32(f));
		}

		/**
		 * Creates an Fp64 that matches the provided double
		 * value closely. Casting the resulting Fp64 back
		 * to a double carries no guarantees that the value
		 * will be identical to when it was initially provided.
		 */
		inline static Fp64 FromDouble(double d)
		{
			return Fp64(fpc::fp64_from_f64(d));
		}

		/**
		 * Creates an Fp64 that matches the provided 32-bit
		 * int exactly. Casting the resulting Fp64 back to
		 * an int32_t is guaranteed to return a value
		 * identical to the initially provided value.
		 */
		inline static Fp64 FromInt32(int32_t i)
		{
			return Fp64(fpc::fp64_from_i32(i));
		}

		/**
		 * Creates an Fp64 that matches the provided 64-bit
		 * int exactly. Casting the resulting Fp64 back to
		 * an int64_t is guaranteed to return a value
		 * identical to the initially provided value.
		 */
		inline static Fp64 FromInt64(int64_t i)
		{
			return Fp64(fpc::fp64_from_i64(i));
		}

		// Conversions

		inline explicit operator bool() const
		{
			return m_Internal.internal;
		}

		inline explicit operator int32_t() const
		{
			return fpc::fp64_to_i32(m_Internal);
		}

		inline explicit operator int64_t() const
		{
			return fpc::fp64_to_i64(m_Internal);
		}

		inline explicit operator float() const
		{
			return fpc::fp64_to_f32(m_Internal);
		}

		inline explicit operator double() const
		{
			return fpc::fp64_to_f64(m_Internal);
		}

		// Arithmetic operators
		inline friend Fp64 operator+(Fp64 Lhs, Fp64 Rhs);

		inline Fp64 operator-(Fp64 Rhs) const
		{
			return Fp64(fpc::fp64_sub(m_Internal, Rhs.m_Internal));
		}

		inline Fp64 operator-() const
		{
			return Fp64(fpc::Fp64{ -m_Internal.internal });
		}

		inline Fp64 operator*(Fp64 Rhs) const
		{
			return Fp64(fpc::fp64_mul(m_Internal, Rhs.m_Internal));
		}

		inline friend Fp64 operator/(Fp64 Lhs, Fp64 Rhs);

		inline Fp64 operator%(Fp64 Rhs) const
		{
			return Fp64(fpc::fp64_mod(m_Internal, Rhs.m_Internal));
		}

		// Arithmetic assignment operators
		inline Fp64& operator+=(Fp64 Rhs)
		{
			m_Internal = fpc::fp64_add(m_Internal, Rhs.m_Internal);
			return *this;
		}

		inline Fp64& operator-=(Fp64 Rhs)
		{
			m_Internal = fpc::fp64_sub(m_Internal, Rhs.m_Internal);
			return *this;
		}

		inline Fp64& operator*=(Fp64 Rhs)
		{
			m_Internal = fpc::fp64_mul(m_Internal, Rhs.m_Internal);
			return *this;
		}

		inline Fp64& operator/=(Fp64 Rhs)
		{
			m_Internal = fpc::fp64_div(m_Internal, Rhs.m_Internal);
			return *this;
		}

		inline Fp64& operator%=(Fp64 Rhs)
		{
			m_Internal = fpc::fp64_mod(m_Internal, Rhs.m_Internal);
			return *this;
		}

		// Comparison operators
		inline bool operator==(const Fp64& Rhs)
		{
			return m_Internal.internal == Rhs.m_Internal.internal;
		}

		inline bool operator!=(const Fp64& Rhs) 
		{
			return !(*this == Rhs);
		}

		inline bool operator>(const Fp64& Rhs) const
		{
			return m_Internal.internal > Rhs.m_Internal.internal;
		}

		inline bool operator>=(const Fp64& Rhs) const
		{
			return m_Internal.internal >= Rhs.m_Internal.internal;
		}

		inline bool operator<(const Fp64& Rhs) const
		{
			return m_Internal.internal < Rhs.m_Internal.internal;
		}

		inline bool operator<=(const Fp64& Rhs) const
		{
			return m_Internal.internal <= Rhs.m_Internal.internal;
		}

		// Arithmetic functions
		inline static Fp64 Sqrt(Fp64 F, Fp64 Precision)
		{
			return Fp64(fpc::fp64_sqrt(F.m_Internal, Precision.m_Internal));
		}

		inline static Fp64 EasySqrt(Fp64 F)
		{
			return Fp64(fpc::fp64_easy_sqrt(F.m_Internal));
		}

		inline static Fp64 Reciprocal_Sqrt(Fp64 F, uint32_t Iterations)
		{
			return Fp64(fpc::fp64_reciprocal_sqrt(F.m_Internal, Iterations));
		}

		inline static Fp64 Pow(Fp64 F, uint32_t Exponent)
		{
			return Fp64(fpc::fp64_pow(F.m_Internal, Exponent));
		}

		inline static Fp64 Sin(Fp64 F)
		{
			return Fp64(fpc::fp64_sin(F.m_Internal));
		}

		inline static Fp64 Cos(Fp64 F)
		{
			return Fp64(fpc::fp64_cos(F.m_Internal));
		}

		inline Fp64 Half() const
		{
			return Fp64(fpc::fp64_half(m_Internal));
		}

		inline Fp64 Double() const
		{
			return Fp64(fpc::fp64_double(m_Internal));
		}

		/**
		 * Returns the internal 64-bit int used to
		 * represent the fixed point value. Note that
		 * in future versions the scale may change, so
		 * manipulate this value with caution.
		 */
		inline int64_t InternalRepresentation() const
		{
			return m_Internal.internal;
		}
	};
	//Friend funcs
	inline Fp64 operator+(Fp64 Lhs, Fp64 Rhs)
	{
		return Fp64(fpc::fp64_add(Lhs.m_Internal, Rhs.m_Internal));
	}
	inline Fp64 operator/(Fp64 Lhs, Fp64 Rhs)
	{
		return Fp64(fpc::fp64_div(Lhs.m_Internal, Rhs.m_Internal));
	}

} // namespace fp64

#endif // __cplusplus

#endif // FP_MATH_DY_H
