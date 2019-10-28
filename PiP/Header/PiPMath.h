#ifndef PIP_MATH
#define PIP_MATH

#include <cmath>

#include "fp_math.h"

#define USE_FIXEDPOINT 1

//Inline Base Math, Vector, Matrix, Quaternion library
#if USE_FIXEDPOINT
typedef fp64::Fp64 decimal;
#else
typedef float decimal;
#endif

namespace math {

	inline decimal Sqrt(decimal x, int iterations = 3) {
#if USE_FIXEDPOINT
		return fp64::Fp64::Sqrt(x, iterations);
#else
		return sqrt(x);
#endif
	}

	typedef struct Vector2Str {

		decimal x, y;

		Vector2Str()
			: x(0.f), y(0.f)
		{}

		Vector2Str(decimal x, decimal y)
			: x(x), y(y) {
		}

		//Arithmetic operators
		inline Vector2Str operator+(Vector2Str Rhs) const
		{
			return Vector2Str(x + Rhs.x, y + Rhs.y);
		}

		inline Vector2Str operator-(Vector2Str Rhs) const
		{
			return Vector2Str(x - Rhs.x, y - Rhs.y);
		}

		inline Vector2Str operator*(decimal scalar) const
		{
			return Vector2Str(x * scalar, y * scalar);
		}

		inline Vector2Str operator/(decimal scalar) const
		{
			return Vector2Str(x / scalar, y / scalar);
		}

		// Arithmetic assignment operators
		inline Vector2Str& operator+=(Vector2Str Rhs)
		{
			x += Rhs.x;
			y += Rhs.y;
			return *this;
		}

		inline Vector2Str& operator-=(Vector2Str Rhs)
		{
			x -= Rhs.x;
			y -= Rhs.y;
			return *this;
		}

		inline Vector2Str& operator*=(decimal Rhs)
		{
			x *= Rhs;
			y *= Rhs;
			return *this;
		}

		inline Vector2Str& operator/=(decimal Rhs)
		{
			x /= Rhs;
			y /= Rhs;
			return *this;
		}

		inline decimal Length() {
			return Sqrt(x * x + y * y);
		}

		inline Vector2Str Normalize() {
			decimal length = Length();
			x /= length;
			y /= length;
			return *this;
		}
		
		inline static decimal Dot(Vector2Str v1, Vector2Str v2) {
			return v1.x * v2.x + v1.y * v2.y;
		}
	}Vector2;
	
	typedef struct Vector4Str {
		decimal x, y, z, w;
		
		Vector4Str( decimal x = 0.f, decimal y = 0.f, decimal z = 0.f, decimal w = 0.f) 
			: x(0.f), y(0.f), z(0.f), w(0.f)
		{}

	}Vector4;
	//Column major for OpenGL
	typedef struct MatrixStr {

		decimal m[4][4];

		MatrixStr() 
			: m{ 
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
		}
		{}

		MatrixStr(Vector4 c1, Vector4 c2, Vector4 c3, Vector4 c4) {
			m[0][0] = c1.x;
			m[1][0] = c1.y;
			m[2][0] = c1.z;
			m[3][0] = c1.w;

			m[0][1] = c2.x;
			m[1][1] = c2.y;
			m[2][1] = c2.z;
			m[3][1] = c2.w;

			m[0][2] = c3.x;
			m[1][2] = c3.y;
			m[2][2] = c3.z;
			m[3][2] = c3.w;

			m[0][3] = c4.x;
			m[1][3] = c4.y;
			m[2][3] = c4.z;
			m[3][3] = c4.w;
		}
		inline static MatrixStr Identity() {
		
		}
	}Matrix;
}
#endif // PIP_MATH

