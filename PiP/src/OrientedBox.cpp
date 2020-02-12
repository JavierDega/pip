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
	Vector2 aToB = rb2->m_position - m_position;
	Vector2 rotExtents = m_halfExtents.Rotate(m_rotation);
	Vector2 rotExtents2 = rb2->m_halfExtents.Rotate(rb2->m_rotation);
	//Possibly add ref arguments to retrieve contact data (amount of penetration,..)
	decimal minPen;
	Vector2 minAxis;
	Vector2 axis;
	decimal penetration;
	SatCollision collisionType;
	//rb1's axii
	axis = Vector2(1, 0).Rotate(m_rotation);
	if (TestAxis(axis, m_position, rb2->m_position, rotExtents,
		rotExtents2, penetration)) {
		//Store penetration and axis
		minPen = penetration;
		minAxis = axis;
		collisionType = SatCollision::OBJ1X;
	}
	else return false;
	axis = Vector2(0, 1).Rotate(m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents,
			rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ1Y;
		}
	} 
	else return false;
	//rb2's axii
	axis = Vector2(1, 0).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents,
		rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2X;
		}
	} 
	else return false;
	axis = Vector2(0, 1).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents,
		rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2Y;
		}
	}
	else return false;

	//#TODO: Contact retrieval
	//We may need to know which face the axis comes from, as well as the side planes, so that we can build a plane and clip the incident face against it

	switch (collisionType) {
	case SatCollision::OBJ1X: 
	{
		//Use relative pos to know which side of face
		//Build reference planes
		Vector2 n1 = minAxis;
		Vector2 n2 = -n1;
		//Side planes
		Vector2 n3 = Vector2(1, 0).Rotate(m_rotation);
		Vector2 n4 = -n3;

		decimal offSet1 = (m_position + Vector2(rotExtents.x, 0)).Dot(n1);
		decimal offSet2 = (m_position + Vector2(-rotExtents.x, 0)).Dot(n2);

		decimal offSet3 = (m_position + Vector2(0, rotExtents.y)).Dot(n3);
		decimal offSet4 = (m_position + Vector2(0, -rotExtents.y)).Dot(n4);

		Vector2 box2Points[4];
		box2Points[0] = rb2->m_position + rotExtents2;
		box2Points[1] = rb2->m_position + Vector2( -rotExtents2.x, rotExtents2.y );
		box2Points[2] = rb2->m_position - rotExtents2;
		box2Points[3] = rb2->m_position + Vector2( rotExtents2.x, -rotExtents2.y );

		//We have data for four planes
		if (minAxis.Dot(aToB) > 0) 
		{
			//Right side of A
		}
		else 
		{
			//Left side of A
		}
	}
	break;
	case SatCollision::OBJ1Y:
	{
	}
	break;
	}
	manifold.rb1 = this;
	manifold.rb2 = rb2;
	return true;
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

bool OrientedBox::TestAxis(math::Vector2 axis, math::Vector2 pos1, math::Vector2 pos2, math::Vector2 rotExtents, math::Vector2 rotExtents2, decimal& penetration)
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
		else {
			//Retrieve important info, like penetration amount on such axis
			penetration = pos1Axis + max - (pos2Axis - min2);
			return true;
		}
	}
	else 
	{
		if (pos2Axis + max2 < pos1Axis - min)
		{
			//Separating axis
			return false;
		}
		else {
			penetration = pos2Axis + max2 - (pos1Axis - min);
			return true;
		}
	}
}
