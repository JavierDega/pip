
#include "Capsule.h"

using namespace math;

Capsule::Capsule(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, decimal length, decimal radius)
	: Rigidbody(pos, rot, vel, angVel, accel, mass), m_length(length), m_radius(radius)
{
}

Capsule::~Capsule()
{
}

bool Capsule::ComputeIntersect(Rigidbody* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool Capsule::IntersectWith(Circle* rb2, Manifold& manifold)
{
	return false;
}

bool Capsule::IntersectWith(Capsule* rb2, Manifold& manifold)
{
	return false;
}

decimal Capsule::ComputeSweep(Rigidbody* rb2, decimal dt, Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal Capsule::SweepWith(Circle* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal Capsule::SweepWith(Capsule* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}
