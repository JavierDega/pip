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

		static Vector4 operator * (MatrixStr m1, Vector4 v1) {
			Vector4 newVec =  Vector4();
			newVec.x = v1.x * m1.m[0][0] + v1.y * m1.m[0][1] + v1.z * m1.m[0][2] + v1.w * m1.m[0][3];
			newVec.y = v1.x * m1.m[1][0] + v1.y * m1.m[1][1] + v1.z * m1.m[1][2] + v1.w * m1.m[1][3];
			newVec.z = v1.x * m1.m[2][0] + v1.y * m1.m[2][1] + v1.z * m1.m[2][2] + v1.w * m1.m[2][3];
			newVec.w = v1.x * m1.m[3][0] + v1.y * m1.m[3][1] + v1.z * m1.m[3][2] + v1.w * m1.m[3][3];
			return newVec;
		}

		static MatrixStr operator * (MatrixStr m1, MatrixStr m2) {

			//First one's columns (of a row) times the second one's rows (of a column)
			MatrixStr m3 = MatrixStr();
			//Result of first column
			m3.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] + m1.m[0][3] * m2.m[3][0];
			m3.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] + m1.m[1][3] * m2.m[3][0];
			m3.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] + m1.m[2][3] * m2.m[3][0];
			m3.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] + m1.m[3][3] * m2.m[3][0];

			//Seguir con segunda tercera cuarta y quinta
			//Result of second column
			m3.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] + m1.m[0][3] * m2.m[3][1];
			m3.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] + m1.m[1][3] * m2.m[3][1];
			m3.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] + m1.m[2][3] * m2.m[3][1];
			m3.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] + m1.m[3][3] * m2.m[3][1];

			//Result of third column
			m3.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] + m1.m[0][3] * m2.m[3][2];
			m3.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] + m1.m[1][3] * m2.m[3][2];
			m3.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] + m1.m[2][3] * m2.m[3][2];
			m3.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] + m1.m[3][3] * m2.m[3][2];

			//Result of fourth column
			m3.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] + m1.m[0][3] * m2.m[3][3];
			m3.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] + m1.m[1][3] * m2.m[3][3];
			m3.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] + m1.m[2][3] * m2.m[3][3];
			m3.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] + m1.m[3][3] * m2.m[3][3];

			/*float sum = 0;
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
			rv.values[0, 3] = -values[0, 3];
			rv.values[1, 3] = -values[1, 3];
			rv.values[2, 3] = -values[2, 3];

			return rv;

		}

		public Matrix4by4 inverseScale() {

			Matrix4by4 rv = Identity;
			rv.values[0, 0] = 1.0f / values[0, 0];
			rv.values[1, 1] = 1.0f / values[1, 1];
			rv.values[2, 2] = 1.0f / values[2, 2];

			return rv;
		}

		public Matrix4by4 inverseRotation() {


			return new Matrix4by4(GetRow(0), GetRow(1), GetRow(2), GetRow(3));
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
}
#endif // PIP_MATH

