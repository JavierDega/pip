#include "Rigidbody.h"

using namespace PipMath;

Rigidbody::Rigidbody(Vector2 pos, decimal rot, Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic, decimal kFriction)
	: m_position(pos), m_rotation(rot), m_velocity(vel), m_angularVelocity(angVel), m_mass(mass), m_e(e), m_isKinematic(isKinematic),
	m_kFriction(kFriction), m_isSleeping(false), m_timeInSleep(0.f), m_inertia(0.f), m_prevPos(), m_prevRot(), m_acceleration(), m_angularAccel()
{
}

Rigidbody::~Rigidbody()
{
}

