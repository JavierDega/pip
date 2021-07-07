#pragma once

#include "Rigidbody.h"
#include "PipMath.h"

class Circle :
	public Rigidbody
{
public:
	Circle(decimal rad = 1.0f, PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.0f, PipMath::Vector2 vel = PipMath::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	~Circle();
	virtual bool IntersectWith(PipMath::Vector2 topRight, PipMath::Vector2 bottomLeft) override;
	virtual bool IntersectWith(Rigidbody* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, PipMath::Manifold& manifold) override;

	decimal m_radius;
};
