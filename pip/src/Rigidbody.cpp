#include "Rigidbody.h"

using namespace PipMath;

Rigidbody::Rigidbody(Vector2 pos, decimal rot, Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
	: m_position(pos), m_rotation(rot), m_velocity(vel), m_angularVelocity(angVel), m_mass(mass), m_e(e), m_isKinematic(isKinematic), 
	m_isSleeping(false), m_timeInSleep(0.f), m_inertia(0.f), m_prevPos()
{
}

Rigidbody::~Rigidbody()
{
}

