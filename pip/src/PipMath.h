#pragma once

#include <cmath>
#include <iostream>

#include "fp_math.h"

#define PIP_TEST_EPSILON 0.00001f
#define PIP_SLEEP_DELTA 0.002f*0.002f
#define PIP_USE_FIXEDPOINT 0
#define PIP_PI 3.14159265f
#define PIP_DEG2RAD (PIP_PI)/180
#define PIP_RAD2DEG 180/(PIP_PI)
 
//Inline Base Math, Vector, Matrix, Quaternion library
#if PIP_USE_FIXEDPOINT
typedef fp64::Fp64 decimal;
#else
typedef float decimal;
#endif

class Rigidbody;//Manifold needs fwdecl

namespace PipMath 
{

	inline decimal Abs(decimal x) 
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::Abs(x);
#else 
		return std::abs(x);
#endif
	}

	inline decimal Min(decimal x, decimal max) 
	{
		return x < max ? x : max;
	}

	inline decimal Max(decimal x, decimal min) 
	{
		return x > min ? x : min;
	}

	inline decimal Clamp(decimal x, decimal min, decimal max) 
	{
#if PIP_USE_FIXEDPOINT
		return Max(min, Min(x, max));
#else
		return fmaxf(min, fminf(x, max));
#endif
	}

	inline decimal Sqrt(decimal x) 
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::EasySqrt(x);
#else
		return sqrt(x);
#endif
	}

	inline decimal Pow(decimal x, unsigned short exponent) 
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::Pow(x, exponent);
#else
		return powf(x, exponent);
#endif
	}

	inline decimal Cos(decimal rad) 
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::Cos(rad);
#else
		return cos(rad);
#endif
	}

	inline decimal Sin(decimal rad) 
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::Sin(rad);
#else 
		return sin(rad);
#endif
	}

	inline decimal Tan(decimal rad)
	{
#if PIP_USE_FIXEDPOINT
		return fp64::Fp64::Sin(rad) / fp64::Fp64::Cos(rad);
#else
		return tan(rad);
#endif
	}
	
	struct Vector3;
	struct Vector2 
	{

		decimal x, y;

		Vector2()
			: x(0.f), y(0.f)
		{
		}

		Vector2(decimal x, decimal y)
			: x(x), y(y) 
		{
		}

		// Conversions
		inline explicit operator bool() const
		{
			return x != 0 || y != 0;
		}

		inline Vector3 ToVector3();

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
		// Comparison operators
		inline bool operator==(const Vector2& Rhs) const
		{
			return x == Rhs.x && y == Rhs.y;
		}

		inline bool EqualsEps(const Vector2& Rhs, decimal epsilon) const
		{
			return (Abs(x - Rhs.x) + Abs(y - Rhs.y)) < epsilon;
		}

		inline bool operator!=(const Vector2& Rhs) const
		{
			return !(*this == Rhs);
		}

		inline decimal Length()
		{
			return Sqrt(x * x + y * y);
		}

		inline Vector2 Normalize() 
		{
			decimal length = Length();
			x /= length;
			y /= length;
			return *this;
		}
		
		inline Vector2 Normalized() 
		{
			decimal length = Length();
			return Vector2(x / length, y / length);
		}
		
		inline decimal LengthSqr()
		{
			return this->Dot(*this);
		}

		inline Vector2 Perp() 
		{
			return Vector2(-y, x);
		}
		//Rotate a point about the origin
		inline Vector2 Rotated(decimal rad) const 
		{
			Vector2 copy = *this;
			return copy.Rotate(rad);
		}

		inline Vector2 Rotate(decimal rad) 
		{
			decimal x2 = x * Cos(rad) - y * Sin(rad);
			decimal y2 = y * Cos(rad) + x * Sin(rad);
			x = x2;
			y = y2;
			return *this;
		}

		inline decimal Dot(Vector2 v2) 
		{
			return x * v2.x + y * v2.y;//Same as LengthSqr if passing same vector
		}

		inline decimal Cross(const Vector2& v2) const
		{
			return (x * v2.y) - (y * v2.x);
		}
	};


	inline Vector2 operator*( const Vector2& v, const decimal& scalar ) 
	{
		return Vector2(v.x * scalar, v.y * scalar);
	}

	inline Vector2 operator*( const decimal& scalar, const Vector2& v )
	{
		return v * scalar;
	}

	inline std::ostream& operator << (std::ostream& out, const Vector2& v)
	{
		out << "x(" << (double)v.x << ")" << " y(" << (double)v.y << ")";
		return out;
	}
	
#if PIP_USE_FIXEDPOINT
	inline std::ostream& operator << (std::ostream& out, const decimal& v)
	{
		out << (double)v;
		return out;
	}
#else
#endif

	//Return point in segment ab closest to point p
	inline Vector2 ClosestPtToSegment(Vector2 a, Vector2 b, Vector2 p)
	{
		Vector2 ab = b - a;
		Vector2 ap = p - a;
		Vector2 bp = p - b;
		//std::cout << "PiP - Log: ClosestPtToSegment: ab " << ab << std::endl;
		//std::cout << "PiP - Log: ClosestPtToSegment: ap " << ap << std::endl;
		//Case 1
		if (ab.Dot(ap) <= 0) 
		{
			return a;
		}
		//Case2
		else if (-ab.Dot(bp) <= 0) 
		{
			return b;
		}
		else 
		{
			//Dot project
			ab.Normalize();
			return  a + (ab * ab.Dot(ap));
		}
	}
	//Point, plane normal, plane dist to origin along n
	inline decimal DistPtToPlane(Vector2 p, Vector2 n, decimal dist)
	{
		n.Normalize();
		Vector2 q = n * dist;//Plane's centre
		Vector2 planeToP = p - q;
		return planeToP.Dot(n);
	}

	struct Vector3 
	{

		decimal x, y, z;

		Vector3()
			: x(0), y(0), z(0)
		{
		}

		Vector3(decimal x, decimal y, decimal z)
			: x(x), y(y), z(z) 
		{
		}

		// Conversions
		inline explicit operator bool() const
		{
			return x != 0 || y != 0 || z!=0;
		}

		inline Vector2 ToVector2() const
		{
			return Vector2(x, y);//Lose Z data
		}

		//Arithmetic operators
		inline Vector3 operator+(Vector3 Rhs) const
		{
			return Vector3(x + Rhs.x, y + Rhs.y, z + Rhs.z);
		}

		inline Vector3 operator-(Vector3 Rhs) const
		{
			return Vector3(x - Rhs.x, y - Rhs.y, z - Rhs.z);
		}

		inline Vector3 operator-() const
		{
			return Vector3(-x, -y, -z);
		}

		inline friend Vector3 operator*(const Vector3& v, const decimal& scalar);

		inline friend Vector3 operator*(const decimal& scalar, const Vector3& v);

		inline Vector3 operator/(const decimal& scalar) const
		{
			return Vector3(x / scalar, y / scalar, z / scalar);
		}

		// Arithmetic assignment operators
		inline Vector3& operator+=(Vector3 Rhs)
		{
			*this = *this + Rhs;
			return *this;
		}

		inline Vector3& operator-=(Vector3 Rhs)
		{
			*this = *this - Rhs;
			return *this;
		}

		inline Vector3& operator*=(decimal Rhs)
		{
			*this = *this * Rhs;
			return *this;
		}

		inline Vector3& operator/=(decimal Rhs)
		{
			*this = *this / Rhs;
			return *this;
		}
		// Comparison operators
		inline bool operator==(const Vector3& Rhs) const
		{
			return x == Rhs.x && y == Rhs.y && z == Rhs.z;
		}

		inline bool operator!=(const Vector3& Rhs) const
		{
			return !(*this == Rhs);
		}

		inline decimal Length() {
			return Sqrt(x * x + y * y + z * z);
		}

		inline Vector3 Normalize()
		{
			decimal length = Length();
			x /= length;
			y /= length;
			z /= length;
			return *this;
		}

		inline Vector3 Normalized() 
		{
			decimal length = Length();
			return Vector3(x / length, y / length, z / length);
		}

		inline decimal LengthSqr() 
		{
			return this->Dot(*this);
		}

		inline Vector3 RotateAxisAngle(Vector3 axis, decimal rad)
		{
			//axis must be unit vector;
			*this = *this * Cos(rad) + axis.Cross(*this) * Sin(rad) + ((decimal)1 - Cos(rad)) * axis.Dot(*this) * axis;
			return *this;
		}

		//Rotate a point about the origin
		inline Vector3 RotatedAxisAngle(Vector3 axis, decimal rad) const
		{
			Vector3 copy = *this;
			return copy.RotateAxisAngle(axis, rad);
		}

		inline decimal Dot(Vector3 v2) 
		{
			return x * v2.x + y * v2.y + z * v2.z;//Same as LengthSqr
		}

		inline Vector3 Cross(const Vector3& v2) const
		{
			return Vector3(y * v2.z - z * v2.y, z * v2.x - x * v2.z, x * v2.y - y * v2.x);
		}
	};

	inline Vector3 Vector2::ToVector3()
	{
		return Vector3(x, y, 0);
	}

	inline Vector3 operator*(const Vector3& v, const decimal& scalar) 
	{
		return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
	}

	inline Vector3 operator*(const decimal& scalar, const Vector3& v) 
	{
		return v * scalar;
	}

	//Quaternions
	typedef struct Vector4Str 
	{
		decimal x, y, z, w;
		
		Vector4Str( decimal x = 0.f, decimal y = 0.f, decimal z = 0.f, decimal w = 0.f) 
			: x(0.f), y(0.f), z(0.f), w(0.f)
		{
		}

	}Vector4;
	//Column major for OpenGL
	typedef struct MatrixStr 
	{

		decimal m[4][4];

		MatrixStr() 
			: m{ 
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0}
		}
		{}

		MatrixStr(Vector4 c1, Vector4 c2, Vector4 c3, Vector4 c4) 
		{
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
		inline Vector4 GetColumn(int index) 
		{
			Vector4 v4 = Vector4(m[index][0], m[index][1], m[index][2], m[index][3]);
			return v4;
		}

		inline Vector4 GetRow(int index) 
		{
			Vector4 v4 = Vector4(m[0][index], m[1][index], m[2][index], m[3][index]);
			return v4;
		}

		inline static MatrixStr Identity() 
		{
			MatrixStr m = MatrixStr();
			m.m[0][0] = 1;
			m.m[1][1] = 1;
			m.m[2][2] = 1;
			m.m[3][3] = 1;
			return m;
		}

		Vector4 operator * (Vector4 v1)
		{
			Vector4 newVec =  Vector4();
			newVec.x = v1.x * m[0][0] + v1.y * m[0][1] + v1.z * m[0][2] + v1.w * m[0][3];
			newVec.y = v1.x * m[1][0] + v1.y * m[1][1] + v1.z * m[1][2] + v1.w * m[1][3];
			newVec.z = v1.x * m[2][0] + v1.y * m[2][1] + v1.z * m[2][2] + v1.w * m[2][3];
			newVec.w = v1.x * m[3][0] + v1.y * m[3][1] + v1.z * m[3][2] + v1.w * m[3][3];
			return newVec;
		}

		MatrixStr operator * (MatrixStr m2)
		{

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

		inline MatrixStr inverseTranslation() 
		{
			MatrixStr rv = Identity();
			rv.m[0][3] = -m[0][3];
			rv.m[1][3] = -m[1][3];
			rv.m[2][3] = -m[2][3];
			return rv;
		}

		inline MatrixStr inverseScale() 
		{

			MatrixStr rv = Identity();
			rv.m[0][0] = (decimal)1.0f / m[0][0];
			rv.m[1][1] = (decimal)1.0f / m[1][1];
			rv.m[2][2] = (decimal)1.0f / m[2][2];

			return rv;
		}
		//Transpose
		inline MatrixStr inverseRotation() 
		{
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
	struct Manifold 
	{
		int numContactPoints;
		decimal penetration;
		PipMath::Vector2 normal;
		PipMath::Vector2 contactPoints[2] = { Vector2(0,0), Vector2(0,0) };//Obb to Obb may use 2 contact points (If incident face complete interpenetrates reference face)
		Rigidbody* rb1;
		Rigidbody* rb2;

		Manifold() 
		{
			numContactPoints = 0;
			penetration = 0.0f;
			normal = Vector2{ 0.0f, 0.0f };
			rb1 = nullptr;
			rb2 = nullptr;
		}
	};
}

