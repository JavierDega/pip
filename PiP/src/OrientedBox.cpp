#include "OrientedBox.h"
#include "Circle.h"
#include "Capsule.h"

using namespace math;

OrientedBox::OrientedBox(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass,
	bool isKinematic, math::Vector2 halfExtents)
	: Rigidbody(pos, rot, vel, angVel, accel, mass, isKinematic), m_halfExtents(halfExtents)
{
	//Find inertia tensor formula for an oriented box (Derived from capsule's)
	m_inertia = m_mass * (Pow(m_halfExtents.x * 2, 2) + Pow(m_halfExtents.y * 2, 2)) / 12;
}

OrientedBox::~OrientedBox()
{
}

bool OrientedBox::ComputeIntersect(Rigidbody* rb2, math::Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Circle* rb2, math::Manifold& manifold)
{
	//ClosestPtCircleToObb query
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Capsule* rb2, math::Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(OrientedBox* rb2, math::Manifold& manifold)
{
	return false;
}

decimal OrientedBox::ComputeSweep(Rigidbody* rb2, decimal dt, math::Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal OrientedBox::SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}
