#pragma once
#include "Rigidbody.h"

class Capsule :
	public Rigidbody
{
public:
	Capsule(decimal length = 1.0f, decimal radius = 1.0f, math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	~Capsule();
	virtual bool IntersectWith(math::Vector2 topRight, math::Vector2 bottomLeft) override;
	virtual bool IntersectWith(Rigidbody* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, math::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold) override;

	decimal m_length;
	decimal m_radius;
};
