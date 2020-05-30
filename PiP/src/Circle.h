#pragma once
#include "Rigidbody.h"

class Circle :
	public Rigidbody
{
public:
	Circle(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false, decimal rad = 1.0f);
	~Circle();
	virtual bool IntersectWith(Rigidbody* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, math::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold) override;

	decimal m_radius;
};
