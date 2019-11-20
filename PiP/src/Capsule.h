#pragma once
#include "Rigidbody.h"

class Capsule :
	public Rigidbody
{
public:
	Capsule(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, math::Vector2 accel = math::Vector2(), decimal mass = 1.0f, decimal length = 1.0f, decimal radius = 1.0f);
	~Capsule();
	virtual math::Manifold ComputeIntersect(Rigidbody* rb2, decimal dt) override;
	virtual math::Manifold IntersectWith(Circle* rb2, decimal dt) override;
	virtual math::Manifold IntersectWith(Capsule* rb2, decimal dt) override;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt) override;

	decimal m_length;
	decimal m_radius;
};
