#pragma once
#include "PiPMath.h"

class Rigidbody
{
public:
	Rigidbody(math::Vector2 pos = math::Vector2(), decimal rot = (decimal)0.f, math::Vector2 vel = math::Vector2(),
		decimal angVel = (decimal)0.f, math::Vector2 accel = math::Vector2(), decimal rad = (decimal)1.f,
		decimal mass = (decimal)1.f);
	~Rigidbody();

	//They're all spheres for now
	math::Vector2 m_position;
	decimal m_rotation;
	math::Vector2 m_velocity;
	decimal m_angularVelocity;
	math::Vector2 m_acceleration;
	//Sphere
	decimal m_radius;
	decimal m_mass;
};
