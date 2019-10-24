#pragma once
#include "VectorArith.h"

class Rigidbody
{
public:
	Rigidbody(Vector2 pos = Vector2(), decimal rot = (decimal)0.f, Vector2 vel = Vector2(), decimal angVel = (decimal)0.f, Vector2 accel = Vector2(), decimal rad = (decimal)1.f,
		decimal mass = (decimal)1.f);
	~Rigidbody();

	//They're all spheres for now
	Vector2 m_position;
	decimal m_rotation;
	Vector2 m_velocity;
	decimal m_angularVelocity;
	Vector2 m_acceleration;
	//Sphere
	decimal m_radius;
	decimal m_mass;
};
