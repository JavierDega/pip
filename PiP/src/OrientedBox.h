
#include "Rigidbody.h"
class OrientedBox :
	public Rigidbody
{
public:
	OrientedBox(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, math::Vector2 accel = math::Vector2(), decimal mass = 1.0f, bool isKinematic = false,
		math::Vector2 halfExtents = math::Vector2(.5f, .5f));
	~OrientedBox();
	virtual bool ComputeIntersect(Rigidbody* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) override;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) override;

	math::Vector2 m_halfExtents;
};
