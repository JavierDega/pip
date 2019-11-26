
#include "Circle.h"
#include "Capsule.h"

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
	//Get Capsule's AB
	decimal halfLength = rb2->m_length / 2;
	Vector2 a = Vector2{ -halfLength, 0 };
	Vector2 b = Vector2{ halfLength, 0 };
	//Rotate about position
	Vector2 aRot = Vector2();
	aRot.x = a.x * Cos(rb2->m_rotation) - a.y * Sin(rb2->m_rotation);
	aRot.y = a.y * Cos(rb2->m_rotation) + a.x * Sin(rb2->m_rotation);
	Vector2 bRot = Vector2();
	bRot.x = b.x * Cos(rb2->m_rotation) - b.y * Sin(rb2->m_rotation);
	bRot.y = b.y * Cos(rb2->m_rotation) + b.x * Sin(rb2->m_rotation);

	aRot += rb2->m_position;
	bRot += rb2->m_position;

	Vector2 ab = bRot - aRot;
	Vector2 c = m_position;
	decimal rab = m_radius + rb2->m_radius;

	Vector2 closestPt;//In capsule, to sphere
	Vector2 ac = c - aRot;
	Vector2 bc = c - bRot;
	//Case 1
	if (ab.Dot(ac) <= 0) {
		closestPt = aRot;
	}
	//Case2
	else if (-ab.Dot(bc) <= 0) {
		closestPt = bRot;
	}
	else {
		//Dot project
		closestPt = a + (ab.Normalize() * ab.Dot(ac));
	}
	Vector2 closestVec = c - closestPt;//Closest pt in caps segment to center of sphere
	if (closestVec.LengthSqr() <= rab * rab){
		//Fill manifold
		Vector2 capsuleEdge = closestPt + closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = c - closestVec * m_radius;
		//manifold.contactPoint 
		return false;
	}
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
