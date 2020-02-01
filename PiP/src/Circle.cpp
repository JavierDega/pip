
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

//https://en.wikipedia.org/wiki/List_of_moments_of_inertia

using namespace math;

Circle::Circle(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, bool isKinematic,
	decimal rad)
	: Rigidbody(pos, rot, vel, angVel, accel, mass, isKinematic), m_radius(rad)
{
	m_inertia = m_mass * m_radius * m_radius / 2;//mr^2/2
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
		manifold.normal = -ab.Normalize();//Point to A by convention
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
	a.Rotate(rb2->m_rotation);
	b.Rotate(rb2->m_rotation);
	a += rb2->m_position;
	b += rb2->m_position;
	Vector2 c = m_position;
	decimal rab = m_radius + rb2->m_radius;
	
	Vector2 closestPt = ClosestPtToSegment(a, b, c);//In capsule, to sphere
	Vector2 closestVec = closestPt - c;//Center of sphere to closestpt in caps segment, also normal
	if (closestVec.LengthSqr() <= rab * rab){
		//Fill manifold
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = c + closestVec * m_radius;
		manifold.normal = -closestVec;//Point to A by convention
		manifold.contactPoint = (capsuleEdge + sphereEdge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	return false;
}

bool Circle::IntersectWith(OrientedBox* rb2, math::Manifold& manifold)
{
	//ClosestPtToObb query
	//Set world origin to be box's center, unrotate all, clamp pt to AABB. Rotate point with box and move it to its pos.
	Vector2 p = m_position - rb2->m_position;
	//Consider the box unrotated, and rotate this p by inverse box's rotation
	p.Rotate(-rb2->m_rotation);
	p.x = Clamp(p.x, -rb2->m_halfExtents.x, rb2->m_halfExtents.x);
	p.y = Clamp(p.y, -rb2->m_halfExtents.y, rb2->m_halfExtents.y);
	//Rotate point back to world space, then translate it
	p.Rotate(rb2->m_rotation);
	p += rb2->m_position;
	//p is closest point from sphere to Obb
	//Get manifold info

	Vector2 circleToClosestPt = p - m_position;
	if (circleToClosestPt.LengthSqr() <= m_radius * m_radius) {
		//Assume circle is outside
		manifold.normal = -circleToClosestPt.Normalize();
		manifold.contactPoint = (m_position + m_radius * circleToClosestPt + p) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
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
		decimal realRoot = (root1 <= root2) ? root1 : root2;
		//Get normal, contact point
		Vector2 rb1Pos = m_position + va * realRoot;
		Vector2 rb2Pos = rb2->m_position + vb * realRoot;
		manifold.contactPoint = rb1Pos + (ra / (ra + rb)) * (rb2Pos - rb1Pos);
		manifold.normal = (rb1Pos - rb2Pos).Normalize();//Point to A by convention
		return realRoot;
	}
}

decimal Circle::SweepWith(Capsule* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal Circle::SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}
