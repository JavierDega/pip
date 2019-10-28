#include "Header\Rigidbody.h"

#if USE_FIXEDPOINT
using namespace fp64;
#endif

using namespace math;

Rigidbody::Rigidbody(Vector2 pos, decimal rot, Vector2 vel, decimal angVel, Vector2 accel, decimal rad, decimal mass)
	: m_position(pos), m_rotation(rot), m_velocity(vel), m_angularVelocity(angVel), m_acceleration(accel), m_radius(rad), m_mass(mass)
{
}

Rigidbody::~Rigidbody()
{
}
