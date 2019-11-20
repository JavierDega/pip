
#include "Capsule.h"

using namespace math;

Capsule::Capsule(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, decimal length, decimal radius)
	: Rigidbody(pos, rot, vel, angVel, accel, mass), m_length(length), m_radius(radius)
{
}

Capsule::~Capsule()
{
}

Manifold Capsule::ComputeIntersect(Rigidbody* rb2, decimal dt)
{
	return rb2->IntersectWith(this, dt);
}

Manifold Capsule::IntersectWith(Circle* rb2, decimal dt)
{
	return nullptr;
}

Manifold Capsule::IntersectWith(Capsule* rb2, decimal dt)
{
	return false;
}

decimal Capsule::ComputeSweep(Rigidbody* rb2, decimal dt)
{
	return rb2->SweepWith(this, dt);
}

decimal Capsule::SweepWith(Circle* rb2, decimal dt)
{
	return decimal();
}

decimal Capsule::SweepWith(Capsule* rb2, decimal dt)
{
	return decimal();
}
