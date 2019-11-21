
#include "Circle.h"

using namespace math;

Circle::Circle(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, decimal rad)
	: Rigidbody(pos, rot, vel, angVel, accel, mass), m_radius(rad)
{
}

Circle::~Circle()
{
}

Manifold Circle::ComputeIntersect(Rigidbody* rb2, decimal dt)
{
	return rb2->IntersectWith(this, dt);
}

Manifold Circle::IntersectWith(Circle* rb2, decimal dt)
{
	return Manifold();
}

Manifold Circle::IntersectWith(Capsule* rb2, decimal dt)
{
	return Manifold();
}

decimal Circle::ComputeSweep(Rigidbody* rb2, decimal dt)
{
	return rb2->SweepWith(this, dt);
}

decimal Circle::SweepWith(Circle* rb2, decimal dt)
{
	//https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=2

	//A = A0 + U*VA
	//B = B0 + U*V

	//B(U) - A(U) = ra + rb

	const decimal ra = m_radius;
	const decimal rb = rb2->m_radius;

	const Vector2 va = m_velocity * dt;
	const Vector2 vb = rb2->m_velocity * dt;

	const Vector2 ab = rb2->m_position - m_position;
	const Vector2 vab = vb - va;

	const decimal rab = ra + rb;

	const decimal a = Vector2::Dot(vab, vab);
	const decimal b = (decimal)2.f * Vector2::Dot(vab, ab);
	const decimal c = Vector2::Dot(ab, ab) - rab * rab;

	const decimal q = b * b - (decimal)4.f * a * c;
	if (q < 0) {
		return 0;//No root, no collision
	}
	else {
		const decimal sq = Sqrt(q, 3);
		const decimal d = (decimal)2 * a;
		const decimal root1 = (-b + sq) / d;
		const decimal root2 = (-b - sq) / d;
		if (root1 <= root2) return root1;
		else return root2;
	}
}

decimal Circle::SweepWith(Capsule* rb2, decimal dt)
{
	return decimal();
}
