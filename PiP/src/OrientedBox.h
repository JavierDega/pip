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
	OrientedBox(pipmath::Vector2 halfExtents = pipmath::Vector2(1.f, 1.f), pipmath::Vector2 pos = pipmath::Vector2(),
	 decimal rot = 0.0f, pipmath::Vector2 vel = pipmath::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	~OrientedBox();
	virtual bool IntersectWith(pipmath::Vector2 topRight, pipmath::Vector2 bottomLeft) override;
	virtual bool IntersectWith(Rigidbody* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, pipmath::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Rigidbody* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, pipmath::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, pipmath::Manifold& manifold) override;
private:
	//#possibly apply Strategy design pattern?
	bool TestAxis(pipmath::Vector2 axis, pipmath::Vector2 pos1, pipmath::Vector2 pos2, pipmath::Vector2 rotExtents,
	 pipmath::Vector2 rotExtents2, decimal& penetration);
public:
	pipmath::Vector2 m_halfExtents;
};
