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
	Rigidbody(PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.f, decimal mass = 1.f, decimal e = .8f,
	 bool isKinematic = false, decimal kFriction = 0.1f);
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
	decimal m_prevRot;
	PipMath::Vector2 m_velocity;
	decimal m_angularVelocity;
	PipMath::Vector2 m_acceleration;
	decimal m_angularAccel;
	decimal m_mass;
	decimal m_e;//coefficient of restitution
	decimal m_kFriction;//friction coefficient (kinetic), doubled when object is static. Recommended range: 0.1f<=x<-0.5f 
	decimal m_timeInSleep;
	bool m_isKinematic, m_isSleeping;
	decimal m_inertia;//scalar in 2D, aka 2nd moment of mass, tensor or matrix in 3D
};
