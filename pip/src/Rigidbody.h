#pragma once

#include "pipmath.h"
#include "QuadNode.h"

class Circle;
class Capsule;
class OrientedBox;

enum class BodyType
{
	Circle,
	Capsule,
	Obb
};

class Rigidbody
{
public:
	Rigidbody(pipmath::Vector2 pos = pipmath::Vector2(), decimal rot = (decimal)0.f,
	 pipmath::Vector2 vel = pipmath::Vector2(), decimal angVel = (decimal)0.f, decimal mass = 1.f, decimal e = 1.f,
	 bool isKinematic = false);
	~Rigidbody();
	//Visitor pattern
	virtual bool IntersectWith(pipmath::Vector2 topRight, pipmath::Vector2 bottomLeft) = 0;
	virtual bool IntersectWith(Rigidbody* rb2, pipmath::Manifold& manifold) = 0;
	virtual bool IntersectWith(Circle* rb2, pipmath::Manifold& manifold) = 0;
	virtual bool IntersectWith(Capsule* rb2, pipmath::Manifold& manifold) = 0;
	virtual bool IntersectWith(OrientedBox* rb2, pipmath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, pipmath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Circle* rb2, decimal dt, pipmath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, pipmath::Manifold& manifold) = 0;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, pipmath::Manifold& manifold) = 0;
public:
	BodyType m_bodyType;
	pipmath::Vector2 m_position;
	pipmath::Vector2 m_prevPos;
	decimal m_rotation;//In radians
	pipmath::Vector2 m_velocity;
	decimal m_angularVelocity;
	pipmath::Vector2 m_acceleration;
	decimal m_mass;
	decimal m_e;//coefficient of restitution
	decimal m_timeInSleep;
	bool m_isKinematic, m_isSleeping;
	decimal m_inertia;//Scalar in 2D aka 2nd moment of mass, tensor or matrix in 3D
};
