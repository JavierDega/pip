#include "OrientedBox.h"
#include "Circle.h"
#include "Capsule.h"

using namespace math;

OrientedBox::OrientedBox(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass,
	bool isKinematic, math::Vector2 halfExtents)
	: Rigidbody(pos, rot, vel, angVel, accel, mass, isKinematic), m_halfExtents(halfExtents)
{
	//Find inertia tensor formula for an oriented box (Derived from capsule's)
	m_inertia = m_mass * (Pow(m_halfExtents.x * 2, 2) + Pow(m_halfExtents.y * 2, 2)) / 12;
}

OrientedBox::~OrientedBox()
{
}

bool OrientedBox::ComputeIntersect(Rigidbody* rb2, math::Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Circle* rb2, math::Manifold& manifold)
{
	//ClosestPtCircleToObb query
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Capsule* rb2, math::Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(OrientedBox* rb2, math::Manifold& manifold)
{
	//SAT
	//We only have 4 axis to project to, but we can simplify it by bringing things to one Obb's reference frame
	if (!TestAxis(Vector2(1, 0).Rotate(m_rotation), m_position, rb2->m_position, m_halfExtents.Rotate(m_rotation),
		rb2->m_halfExtents.Rotate(rb2->m_rotation))) return false;
	
	return false;
}

decimal OrientedBox::ComputeSweep(Rigidbody* rb2, decimal dt, math::Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal OrientedBox::SweepWith(Circle* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(Capsule* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}

bool OrientedBox::TestAxis(math::Vector2 axis, math::Vector2 pos1, math::Vector2 pos2, math::Vector2 rotExtents, math::Vector2 rotExtents2)
{
	decimal pos1Axis = pos1.Dot(axis);
	decimal pos2Axis = pos2.Dot(axis);
	Vector2 p[4]{ Vector2(rotExtents), Vector2(-rotExtents.x, rotExtents.y), Vector2(-rotExtents), Vector2(rotExtents.x, -rotExtents.y) };
	Vector2 p2[4]{ Vector2(rotExtents2), Vector2(-rotExtents2.x, rotExtents2.y), Vector2(-rotExtents2), Vector2(rotExtents2.x, -rotExtents2.y) };
	decimal min = 0, max = 0;
	decimal min2 = 0, max2 = 0;

	for (int i = 0; i < 4; i++) {
		decimal projection = p[i].Dot(axis);
		decimal projection2 = p2[i].Dot(axis);
		if (projection < min) min = projection;
		if (projection > max) max = projection;
		if (projection2 < min2) min2 = projection2;
		if (projection2 > max2) max2 = projection2;
	}

	if (pos1Axis <= pos2Axis)
	{
		if (pos1Axis + max < pos2Axis - min2)
		{
			//Separating axis
			return false;
		}
		else return true;
	}
	else 
	{
		if (pos2Axis + max2 < pos1Axis - min)
		{
			//Separating axis
			return false;
		}
		else return true;
	}
}
