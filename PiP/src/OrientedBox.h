
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
	OrientedBox(math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, math::Vector2 accel = math::Vector2(), decimal mass = 1.0f, bool isKinematic = false,
		math::Vector2 halfExtents = math::Vector2(.5f, .5f));
	~OrientedBox();
	virtual bool ComputeIntersect(Rigidbody* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Circle* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(Capsule* rb2, math::Manifold& manifold) override;
	virtual bool IntersectWith(OrientedBox* rb2, math::Manifold& manifold) override;
	virtual decimal ComputeSweep(Rigidbody* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold) override;
	virtual decimal SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold) override;

	//private
	bool TestAxis(math::Vector2 axis, math::Vector2 pos1, math::Vector2 pos2, math::Vector2 rotExtents, math::Vector2 rotExtents2, decimal& penetration);

	math::Vector2 m_halfExtents;
};
