#pragma once

#include "Rigidbody.h"
#include "pipmath.h"

class Capsule :
	public Rigidbody
{
public:
	Capsule(decimal length = 1.0f, decimal radius = 1.0f, pipmath::Vector2 pos = pipmath::Vector2(), decimal rot = 0.0f,
	 pipmath::Vector2 vel = pipmath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	~Capsule();
	virtual bool IntersectWith(pipmath::Vector2 topRight, pipmath::Vector2 bottomLeft) override;
	virtual bool IntersectWith(Rigidbody* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, pipmath::Manifold& manifold) override;

	decimal m_length;
	decimal m_radius;
};
