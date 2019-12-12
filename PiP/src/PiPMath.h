#ifndef PIP_MATH
#define PIP_MATH

#include <cmath>

#include "fp_math.h"

#define USE_FIXEDPOINT 0
#define PI 3.14159265f
#define DEG2RAD (PI)/180
#define RAD2DEG 180/(PI)

//Inline Base Math, Vector, Matrix, Quaternion library
#if USE_FIXEDPOINT
typedef fp64::Fp64 decimal;
#else
typedef float decimal;
#endif

class Rigidbody;//Manifold needs fwdecl

namespace math {

	inline decimal Sqrt(decimal x) {
#if USE_FIXEDPOINT
		return fp64::Fp64::EasySqrt(x);
#else
		return sqrt(x);
#endif
	}

	inline decimal Pow(decimal x, unsigned short exponent) {
#if USE_FIXEDPOINT
		return fp64::Fp64::Pow(x, exponent);
#else
		return powf(x, exponent);
#endif
	}

	inline decimal Cos(decimal rad) {
#if USE_FIXEDPOINT
		return fp64::Fp64::Cos(rad);
#else
		return cos(rad);
#endif
	}

	inline decimal Sin(decimal rad) {
#if USE_FIXEDPOINT
		return fp64::Fp64::Sin(rad);
#else 
		return sin(rad);
#endif
	}
	
	struct Vector2 {

		decimal x, y;

		Vector2()
			: x(0.f), y(0.f)
		{}

		Vector2(decimal x, decimal y)
			: x(x), y(y) {
		}

		//Arithmetic operators
		inline Vector2 operator+(Vector2 Rhs) const
		{
			return Vector2(x + Rhs.x, y + Rhs.y);
		}

		inline Vector2 operator-(Vector2 Rhs) const
		{
			return Vector2(x - Rhs.x, y - Rhs.y);
		}

		inline Vector2 operator-() const
		{
			return Vector2(-x, -y);
		}

		inline friend Vector2 operator*(const Vector2& v, const decimal& scalar);

		inline friend Vector2 operator*(const decimal& scalar, const Vector2& v);

		inline Vector2 operator/(const decimal& scalar) const
		{
			return Vector2(x / scalar, y / scalar);
		}

		// Arithmetic assignment operators
		inline Vector2& operator+=(Vector2 Rhs)
		{
			x += Rhs.x;
			y += Rhs.y;
			return *this;
		}

		inline Vector2& operator-=(Vector2 Rhs)
		{
			x -= Rhs.x;
			y -= Rhs.y;
			return *this;
		}

		inline Vector2& operator*=(decimal Rhs)
		{
			*this = *this * Rhs;
			return *this;
		}

		inline Vector2& operator/=(decimal Rhs)
		{
			return *this = *this / Rhs;
		}

		inline decimal Length() {
			return Sqrt(x * x + y * y);
		}

		inline Vector2 Normalize() {
			decimal length = Length();
			x /= length;
			y /= length;
			return *this;
		}
		
		inline decimal LengthSqr() {
			return this->Dot(*this);
		}

		inline Vector2 Perp() {
			return Vector2(-y, x);
		}

		inline decimal Dot(Vector2 v2) {
			return x * v2.x + y * v2.y;
		}

		inline decimal Cross(const Vector2& v2) const
		{
			return (x * v2.y) - (y * v2.x);
		}
		// Comparison operators
		inline bool operator==(const Vector2& Rhs)
		{
			return x == Rhs.x && y == Rhs.y;
		}

		inline bool operator!=(const Vector2& Rhs)
		{
			return !(*this == Rhs);
		}

	};

	inline Vector2 operator*( const Vector2& v, const decimal& scalar ) {
		return Vector2(v.x * scalar, v.y * scalar);
	}

	inline Vector2 operator*( const decimal& scalar, const Vector2& v ) {
		return v * scalar;
	}

	//Quaternions
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
			m[0][1] = c1.y;
			m[0][2] = c1.z;
			m[0][3] = c1.w;

			m[1][0] = c2.x;
			m[1][1] = c2.y;
			m[1][2] = c2.z;
			m[1][3] = c2.w;

			m[2][0] = c3.x;
			m[2][1] = c3.y;
			m[2][2] = c3.z;
			m[2][3] = c3.w;

			m[3][0] = c4.x;
			m[3][1] = c4.y;
			m[3][2] = c4.z;
			m[3][3] = c4.w;
		}

		//#WARNING ALL THIS MOVED STUFF MAY ORIGINALLY BE IN ROW MAJOR ORDERING (SEE INV TRANSLATION)
		inline Vector4 GetColumn(int index) {
			Vector4 v4 = Vector4(m[index][0], m[index][1], m[index][2], m[index][3]);
			return v4;
		}

		inline Vector4 GetRow(int index) {
			Vector4 v4 = Vector4(m[0][index], m[1][index], m[2][index], m[3][index]);
			return v4;
		}

		inline static MatrixStr Identity() {
			MatrixStr m = MatrixStr();
			m.m[0][0] = 1;
			m.m[1][1] = 1;
			m.m[2][2] = 1;
			m.m[3][3] = 1;
		}

		Vector4 operator * (Vector4 v1) {
			Vector4 newVec =  Vector4();
			newVec.x = v1.x * m[0][0] + v1.y * m[0][1] + v1.z * m[0][2] + v1.w * m[0][3];
			newVec.y = v1.x * m[1][0] + v1.y * m[1][1] + v1.z * m[1][2] + v1.w * m[1][3];
			newVec.z = v1.x * m[2][0] + v1.y * m[2][1] + v1.z * m[2][2] + v1.w * m[2][3];
			newVec.w = v1.x * m[3][0] + v1.y * m[3][1] + v1.z * m[3][2] + v1.w * m[3][3];
			return newVec;
		}

		MatrixStr operator * (MatrixStr m2) {

			//First one's columns (of a row) times the second one's rows (of a column)
			MatrixStr m3 = MatrixStr();
			//Result of first column
			m3.m[0][0] = m[0][0] * m2.m[0][0] + m[0][1] * m2.m[1][0] + m[0][2] * m2.m[2][0] + m[0][3] * m2.m[3][0];
			m3.m[1][0] = m[1][0] * m2.m[0][0] + m[1][1] * m2.m[1][0] + m[1][2] * m2.m[2][0] + m[1][3] * m2.m[3][0];
			m3.m[2][0] = m[2][0] * m2.m[0][0] + m[2][1] * m2.m[1][0] + m[2][2] * m2.m[2][0] + m[2][3] * m2.m[3][0];
			m3.m[3][0] = m[3][0] * m2.m[0][0] + m[3][1] * m2.m[1][0] + m[3][2] * m2.m[2][0] + m[3][3] * m2.m[3][0];

			//Seguir con segunda tercera cuarta y quinta
			//Result of second column
			m3.m[0][1] = m[0][0] * m2.m[0][1] + m[0][1] * m2.m[1][1] + m[0][2] * m2.m[2][1] + m[0][3] * m2.m[3][1];
			m3.m[1][1] = m[1][0] * m2.m[0][1] + m[1][1] * m2.m[1][1] + m[1][2] * m2.m[2][1] + m[1][3] * m2.m[3][1];
			m3.m[2][1] = m[2][0] * m2.m[0][1] + m[2][1] * m2.m[1][1] + m[2][2] * m2.m[2][1] + m[2][3] * m2.m[3][1];
			m3.m[3][1] = m[3][0] * m2.m[0][1] + m[3][1] * m2.m[1][1] + m[3][2] * m2.m[2][1] + m[3][3] * m2.m[3][1];

			//Result of third column
			m3.m[0][2] = m[0][0] * m2.m[0][2] + m[0][1] * m2.m[1][2] + m[0][2] * m2.m[2][2] + m[0][3] * m2.m[3][2];
			m3.m[1][2] = m[1][0] * m2.m[0][2] + m[1][1] * m2.m[1][2] + m[1][2] * m2.m[2][2] + m[1][3] * m2.m[3][2];
			m3.m[2][2] = m[2][0] * m2.m[0][2] + m[2][1] * m2.m[1][2] + m[2][2] * m2.m[2][2] + m[2][3] * m2.m[3][2];
			m3.m[3][2] = m[3][0] * m2.m[0][2] + m[3][1] * m2.m[1][2] + m[3][2] * m2.m[2][2] + m[3][3] * m2.m[3][2];

			//Result of fourth column
			m3.m[0][3] = m[0][0] * m2.m[0][3] + m[0][1] * m2.m[1][3] + m[0][2] * m2.m[2][3] + m[0][3] * m2.m[3][3];
			m3.m[1][3] = m[1][0] * m2.m[0][3] + m[1][1] * m2.m[1][3] + m[1][2] * m2.m[2][3] + m[1][3] * m2.m[3][3];
			m3.m[2][3] = m[2][0] * m2.m[0][3] + m[2][1] * m2.m[1][3] + m[2][2] * m2.m[2][3] + m[2][3] * m2.m[3][3];
			m3.m[3][3] = m[3][0] * m2.m[0][3] + m[3][1] * m2.m[1][3] + m[3][2] * m2.m[2][3] + m[3][3] * m2.m[3][3];

			/* LOOP APPROACH
			float sum = 0;
			int k = 0;
			for (int i = 0; i <= 3; i++) {
				for (int j = 0; j <= 3; j++) {
					sum = 0;
					for (k = 0; k <= 3; k++) {
						sum = sum + m1.values[i,k] * m2.values[k,j];
					}
					m3.values[i,j] = sum;
					Debug.Log ("Result " + sum);
				}
			}*/
			return m3;
		}

		inline MatrixStr inverseTranslation() {
			MatrixStr rv = Identity();
			rv.m[0][3] = -m[0][3];
			rv.m[1][3] = -m[1][3];
			rv.m[2][3] = -m[2][3];
			return rv;
		}

		inline MatrixStr inverseScale() {

			MatrixStr rv = Identity();
			rv.m[0][0] = (decimal)1.0f / m[0][0];
			rv.m[1][1] = (decimal)1.0f / m[1][1];
			rv.m[2][2] = (decimal)1.0f / m[2][2];

			return rv;
		}
		//Transpose
		inline MatrixStr inverseRotation() {
			return MatrixStr(GetRow(0), GetRow(1), GetRow(2), GetRow(3));
			/*//Matrix transpose

			Matrix4by4 m2 = new Matrix4by4();

			//Create transposed matrix
			m2.values [0, 0] = m1.values [0, 0];
			m2.values [0, 1] = m1.values [1, 0];
			m2.values [0, 2] = m1.values [2, 0];
			m2.values [0, 3] = m1.values [3, 0];

			m2.values [1, 0] = m1.values [0, 1];
			m2.values [1, 1] = m1.values [1, 1];
			m2.values [1, 2] = m1.values [2, 1];
			m2.values [1, 3] = m1.values [3, 1];

			m2.values [2, 0] = m1.values [0, 2];
			m2.values [2, 1] = m1.values [1, 2];
			m2.values [2, 2] = m1.values [2, 2];
			m2.values [2, 3] = m2.values [3, 2];


			//Assign
			m1.values [0, 0] = m2.values [0, 0];
			m1.values [0, 1] = m2.values [0, 1];
			m1.values [0, 2] = m2.values [0, 2];
			m1.values [0, 3] = m2.values [0, 3];

			m1.values [1, 0] = m2.values [1, 0];
			m1.values [1, 1] = m2.values [1, 1];
			m1.values [1, 2] = m2.values [1, 2];
			m1.values [1, 3] = m2.values [1, 3];

			m1.values [2, 0] = m2.values [2, 0];
			m1.values [2, 1] = m2.values [2, 1];
			m1.values [2, 2] = m2.values [2, 2];
			m1.values [2, 3] = m2.values [2, 3];

			m1.values [3, 0] = m2.values [3, 0];
			m1.values [3, 1] = m2.values [3, 1];
			m1.values [3, 2] = m2.values [3, 2];
			m1.values [3, 3] = m2.values [3, 3];
			*/
		}

	}Matrix;
	//Collision info necessary for solver, normal points from objA to B
	struct Manifold {
		decimal penetration;
		math::Vector2 normal;
		math::Vector2 contactPoint;
		Rigidbody* rb1;
		Rigidbody* rb2;

		Manifold() {
			penetration = 0.0f;
			normal = Vector2{ 0.0f, 0.0f };
			contactPoint = Vector2{ 0.0f, 0.0f };
			rb1 = nullptr;
			rb2 = nullptr;
		}
	};
}
#endif // PIP_MATH

