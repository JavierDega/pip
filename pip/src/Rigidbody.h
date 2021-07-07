#pragma once

#include "PipMath.h"
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
	Rigidbody(PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = (decimal)0.f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = (decimal)0.f, decimal mass = 1.f, decimal e = 1.f,
	 bool isKinematic = false);
	~Rigidbody();
	//Visitor pattern
	virtual bool IntersectWith(PipMath::Vector2 topRight, PipMath::Vector2 bottomLeft) = 0;
	virtual bool IntersectWith(Rigidbody* rb2, PipMath::Manifold& manifold) = 0;
	virtual bool IntersectWith(Circle* rb2, PipMath::Manifold& manifold) = 0;
	virtual bool IntersectWith(Capsule* rb2, PipMath::Manifold& manifold) = 0;
	virtual bool IntersectWith(OrientedBox* rb2, PipMath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, PipMath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Circle* rb2, decimal dt, PipMath::Manifold& manifold) = 0;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, PipMath::Manifold& manifold) = 0;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, PipMath::Manifold& manifold) = 0;
public:
	BodyType m_bodyType;
	PipMath::Vector2 m_position;
	PipMath::Vector2 m_prevPos;
	decimal m_rotation;//In radians
	PipMath::Vector2 m_velocity;
	decimal m_angularVelocity;
	PipMath::Vector2 m_acceleration;
	decimal m_mass;
	decimal m_e;//coefficient of restitution
	decimal m_timeInSleep;
	bool m_isKinematic, m_isSleeping;
	decimal m_inertia;//Scalar in 2D aka 2nd moment of mass, tensor or matrix in 3D
};
