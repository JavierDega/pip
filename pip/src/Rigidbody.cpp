#include "Rigidbody.h"

using namespace PipMath;

Rigidbody::Rigidbody(Vector2 pos, decimal rot, Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic, decimal kFriction)
	: m_position(pos), m_prevPos(), m_rotation(rot), m_prevRot(), m_velocity(vel), m_angularVelocity(angVel), m_acceleration(), m_angularAccel(),
	m_mass(mass), m_e(e), m_kFriction(kFriction), m_timeInSleep(0.f), m_isKinematic(isKinematic), m_isSleeping(false), m_inertia(0.f)
{
}

Rigidbody::~Rigidbody()
{
}

