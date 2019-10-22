#pragma once

//Inline Vector, Matrix, Quaternion library
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
		return Vector2Str(x*scalar, y*scalar);
	}

	inline Vector2Str operator/(decimal scalar) const
	{
		return Vector2Str(x / scalar, y / scalar);
	}

	// Arithmetic assignment operators
	inline Vector2Str &operator+=(Vector2Str Rhs)
	{
		x += Rhs.x;
		y += Rhs.y;
		return *this;
	}

	inline Vector2Str &operator-=(Vector2Str Rhs)
	{
		x -= Rhs.x;
		y -= Rhs.y;
		return *this;
	}

	inline Vector2Str &operator*=(decimal Rhs)
	{
		x *= Rhs;
		y *= Rhs;
		return *this;
	}

	inline Vector2Str &operator/=(decimal Rhs)
	{
		x /= Rhs;
		y /= Rhs;
		return *this;
	}

	inline static decimal Dot(Vector2Str v1, Vector2Str v2) {
		return v1.x*v2.x + v1.y*v2.y;
	}
}Vector2;

