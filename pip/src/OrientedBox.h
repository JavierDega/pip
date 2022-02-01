#pragma once

#include "Rigidbody.h"

//May get more complex in the future? All we need for now, just to know which object to clip against which
enum class SatCollision {
	OBJ1,
	OBJ2
};

class OrientedBox :
	public Rigidbody
{
public:
	OrientedBox(PipMath::Vector2 halfExtents = PipMath::Vector2(1.f, 1.f), PipMath::Vector2 pos = PipMath::Vector2(),
	 decimal rot = 0.0f, PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f,
	 decimal e = .8f, bool isKinematic = false, decimal kFriction = 0.03f);
	~OrientedBox();
	virtual bool IntersectWith(PipMath::Vector2 topRight, PipMath::Vector2 bottomLeft) override;
	virtual bool IntersectWith(Rigidbody* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, PipMath::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, PipMath::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, PipMath::Manifold& manifold) override;
private:
	//#possibly apply Strategy design pattern?
	bool TestAxis(PipMath::Vector2 axis, PipMath::Vector2 pos1, PipMath::Vector2 pos2, PipMath::Vector2 points[],
	 PipMath::Vector2 points2[], decimal& penetration);
public:
	PipMath::Vector2 m_halfExtents;
};
