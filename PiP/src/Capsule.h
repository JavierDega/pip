#pragma once
#include "Rigidbody.h"

class Capsule :
	public Rigidbody
{
public:
	Capsule(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, math::Vector2 accel = math::Vector2(), decimal mass = 1.0f, decimal length = 1.0f, decimal radius = 1.0f);
	~Capsule();
	virtual bool ComputeIntersect(Rigidbody* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) override;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) override;

	decimal m_length;
	decimal m_radius;
};
