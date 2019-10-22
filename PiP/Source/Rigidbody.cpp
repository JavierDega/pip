#include "stdafx.h"
#include "Header\Rigidbody.h"

using namespace fp64;

Rigidbody::Rigidbody() 
	: m_position(), m_rotation(0.f), m_velocity(), m_angularVelocity(0.f), m_acceleration(), m_radius(0.f), m_mass(1.f)
{
}


Rigidbody::~Rigidbody()
{
}
