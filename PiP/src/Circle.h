#pragma once
#include "Rigidbody.h"
class Circle :
	public Rigidbody
{
public:
	Circle(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, math::Vector2 accel = math::Vector2(), decimal mass = 1.0f, decimal rad = 1.0f);
	~Circle();
	virtual bool ComputeIntersect(Rigidbody* rb2, decimal dt) override;
	virtual bool IntersectWith(Circle* rb2, decimal dt) override;
	virtual bool IntersectWith(Capsule* rb2, decimal dt) override;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt) override;

	decimal m_radius;
};
