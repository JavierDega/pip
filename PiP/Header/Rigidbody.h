#pragma once
#include "VectorArith.h"

class Rigidbody
{
public:
	Rigidbody();
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
