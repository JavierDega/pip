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
	//
	BodyType GetBodyType();
	decimal GetMass();
	decimal GetInertia();//Retrieve moment of inertia
public:
	PipMath::Vector2 m_position;//Editing directly could put objects inside one another creating degenerate cases prone to crashing
	PipMath::Vector2 m_prevPos;//Used for object sleep detection
	decimal m_rotation;//In radians, editing directly can cause degenerate cases as above
	decimal m_prevRot;//Sleep detection
	PipMath::Vector2 m_velocity;//Main way of moving physics objects through game logic
	decimal m_angularVelocity;//Main way of rotating physics objects
	PipMath::Vector2 m_acceleration;//Editing this directly will only apply it for one frame, accel is reset on Step()
	decimal m_angularAccel;//Same as above
	decimal m_e;//Coefficient of restitution. Recommended range: 0.6f<=x<=0.999..f
	decimal m_kFriction;//friction coefficient (kinetic), doubled when object is static. Recommended range: 0.00..1f<=x<0.5f
	decimal m_timeInSleep;//Used for sleeping objects
	bool m_isKinematic, m_isSleeping;
protected:
	BodyType m_bodyType;
	decimal m_mass;//Cant edit at runtime as it would imply recalculating inertia. Range of  0<x<999...f
	decimal m_inertia;//Calculated off of mass. scalar in 2D, aka 2nd moment of mass, tensor or matrix in 3D
private:

};
