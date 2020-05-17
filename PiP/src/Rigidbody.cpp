#include "Rigidbody.h"

#if USE_FIXEDPOINT
using namespace fp64;
#endif

using namespace math;

Rigidbody::Rigidbody(Vector2 pos, decimal rot, Vector2 vel, decimal angVel, Vector2 accel, decimal mass, bool isKinematic)
	: m_position(pos), m_rotation(rot), m_velocity(vel), m_angularVelocity(angVel), m_acceleration(accel), m_mass(mass), m_isSleeping(false),
	m_timeInSleep(0.02f), m_isKinematic(isKinematic), m_inertia(0.f)
{
}

Rigidbody::~Rigidbody()
{
}

