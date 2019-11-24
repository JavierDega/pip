
#include "Circle.h"

using namespace math;

Circle::Circle(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, decimal rad)
	: Rigidbody(pos, rot, vel, angVel, accel, mass), m_radius(rad)
{
}

Circle::~Circle()
{
}

bool Circle::ComputeIntersect(Rigidbody* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool Circle::IntersectWith(Circle* rb2, Manifold& manifold)
{
	Vector2 ab = rb2->m_position - m_position;
	if (ab.LengthSqr() <= Pow((m_radius + rb2->m_radius), 2)) {
		//Manifold
		manifold.normal = ab.Normalize();
		Vector2 circle1Edge = m_position + ab * m_radius;
		Vector2 circle2Edge = rb2->m_position - ab * rb2->m_radius;
		manifold.contactPoint = circle1Edge + (circle2Edge - circle1Edge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	return false;
}

bool Circle::IntersectWith(Capsule* rb2, Manifold& manifold)
{
	return false;
}

decimal Circle::ComputeSweep(Rigidbody* rb2, decimal dt, Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal Circle::SweepWith(Circle* rb2, decimal dt, Manifold& manifold)
{
	//https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=2

	//A = A0 + U*VA
	//B = B0 + U*V

	//B(U) - A(U) = ra + rb

	const decimal ra = m_radius;
	const decimal rb = rb2->m_radius;

	const Vector2 va = m_velocity * dt;
	const Vector2 vb = rb2->m_velocity * dt;

	Vector2 ab = rb2->m_position - m_position;
	Vector2 vab = vb - va;

	const decimal rab = ra + rb;

	const decimal a = vab.LengthSqr();
	const decimal b = (decimal)2.f * vab.Dot(ab);
	const decimal c = ab.LengthSqr() - rab * rab;

	const decimal q = b * b - (decimal)4.f * a * c;
	if (q < 0) {
		return 0;//No root, no collision
	}
	else {
		const decimal sq = Sqrt(q);
		const decimal d = (decimal)2 * a;
		decimal root1 = (-b + sq) / d;
		decimal root2 = (-b - sq) / d;
		//Take out negatives
		root1 = (decimal)fmax((double)root1, 0);
		root2 = (decimal)fmax((double)root2, 0);
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		if (root1 <= root2) return root1;
		else return root2;
	}
}

decimal Circle::SweepWith(Capsule* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}
