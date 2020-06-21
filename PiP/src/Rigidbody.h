#pragma once
#include "PiPMath.h"

class Circle;
class Capsule;
class OrientedBox;

class Rigidbody
{
public:
	Rigidbody(math::Vector2 pos = math::Vector2(), decimal rot = (decimal)0.f, math::Vector2 vel = math::Vector2(),
		decimal angVel = (decimal)0.f, decimal mass = 1.f, decimal e = 1.f, bool isKinematic = false);
	~Rigidbody();

	//Visitor pattern
	virtual bool IntersectWith(Rigidbody* rb2, math::Manifold& manifold) = 0;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) = 0;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) = 0;
	virtual bool IntersectWith(OrientedBox* rb2, math::Manifold& manifold) = 0;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, math::Manifold& manifold) = 0;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) = 0;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) = 0;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold) = 0;

	math::Vector2 m_position;
	math::Vector2 m_prevPos;
	decimal m_rotation;//In radians
	math::Vector2 m_velocity;
	decimal m_angularVelocity;
	math::Vector2 m_acceleration;
	decimal m_mass;
	decimal m_e;//coefficient of restitution
	decimal m_timeInSleep;
	bool m_isKinematic, m_isSleeping;
	decimal m_inertia;//Scalar in 2D aka 2nd moment of mass, tensor or matrix in 3D
};
