#pragma once
#include "PiPMath.h"

class Circle;
class Capsule;

class Rigidbody
{
public:
	Rigidbody(math::Vector2 pos = math::Vector2(), decimal rot = (decimal)0.f, math::Vector2 vel = math::Vector2(),
		decimal angVel = (decimal)0.f, math::Vector2 accel = math::Vector2(), decimal mass = 1.f);
	~Rigidbody();

	//Visitor pattern
	virtual math::Manifold ComputeIntersect(Rigidbody* rb2, decimal dt) = 0;
	virtual math::Manifold IntersectWith(Circle* rb2, decimal dt) = 0;
	virtual math::Manifold IntersectWith(Capsule* rb2, decimal dt) = 0;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt) = 0;
	virtual decimal SweepWith(Circle* rb2, decimal dt) = 0;
	virtual decimal SweepWith(Capsule* rb2, decimal dt) = 0;

	math::Vector2 m_position;
	decimal m_rotation;
	math::Vector2 m_velocity;
	decimal m_angularVelocity;
	math::Vector2 m_acceleration;
	decimal m_mass;
};
