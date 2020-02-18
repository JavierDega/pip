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
	Vector2 rotExtents = m_halfExtents.Rotated(m_rotation);
	Vector2 rotExtents2 = rb2->m_halfExtents.Rotated(rb2->m_rotation);
	//Possibly add ref arguments to retrieve contact data (amount of penetration,..)
	decimal minPen;
	Vector2 minAxis;
	Vector2 axis;
	decimal penetration;
	SatCollision collisionType;
	//rb1's axii
	axis = Vector2(1, 0).Rotate(m_rotation);
	if (TestAxis(axis, m_position, rb2->m_position, rotExtents, rotExtents2, penetration)) {
		//Store penetration and axis
		minPen = penetration;
		minAxis = axis;
		collisionType = SatCollision::OBJ1;
	}
	else return false;
	axis = Vector2(0, 1).Rotate(m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents, rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ1;
		}
	} 
	else return false;
	//rb2's axii
	axis = Vector2(1, 0).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents, rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2;
		}
	} 
	else return false;
	axis = Vector2(0, 1).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, rotExtents, rotExtents2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2;
		}
	}
	else return false;

	//#TODO: Contact retrieval
	//We may need to know which face the axis comes from, as well as the side planes, so that we can build a plane and clip the incident face against it
	Vector2 planeNormals[4];
	decimal planeDists[4];
	Vector2 boxPoints[4];
	switch (collisionType) {
	case SatCollision::OBJ1: 
	{
		//Build reference planes
		planeNormals[0] = Vector2(1, 0).Rotate(m_rotation);
		planeNormals[1] = -planeNormals[0];
		planeNormals[2] = Vector2(0, 1).Rotate(m_rotation);
		planeNormals[3] = -planeNormals[2];

		planeDists[0] = (m_position + Vector2(m_halfExtents.x, 0).Rotate(m_rotation)).Dot(planeNormals[0]);
		planeDists[1] = (m_position + Vector2(-m_halfExtents.x, 0).Rotate(m_rotation)).Dot(planeNormals[1]);
		planeDists[2] = (m_position + Vector2(0, m_halfExtents.y).Rotate(m_rotation)).Dot(planeNormals[2]);
		planeDists[3] = (m_position + Vector2(0, -m_halfExtents.y).Rotate(m_rotation)).Dot(planeNormals[3]);
		//Points to clip
		boxPoints[0] = rb2->m_position + rotExtents2;
		boxPoints[1] = rb2->m_position + Vector2( -rotExtents2.x, rotExtents2.y );
		boxPoints[2] = rb2->m_position - rotExtents2;
		boxPoints[3] = rb2->m_position + Vector2( rotExtents2.x, -rotExtents2.y );
	}
	break;
	case SatCollision::OBJ2:
	{
		planeNormals[0] = Vector2(1, 0).Rotate(rb2->m_rotation);
		planeNormals[1] = -planeNormals[0];
		planeNormals[2] = Vector2(0, 1).Rotate(rb2->m_rotation);
		planeNormals[3] = -planeNormals[2];

		planeDists[0] = (rb2->m_position + Vector2(rb2->m_halfExtents.x, 0).Rotate(rb2->m_rotation)).Dot(planeNormals[0]);
		planeDists[1] = (rb2->m_position + Vector2(-rb2->m_halfExtents.x, 0).Rotate(rb2->m_rotation)).Dot(planeNormals[1]);
		planeDists[2] = (rb2->m_position + Vector2(0, rb2->m_halfExtents.y).Rotate(rb2->m_rotation)).Dot(planeNormals[2]);
		planeDists[3] = (rb2->m_position + Vector2(0, -rb2->m_halfExtents.y).Rotate(rb2->m_rotation)).Dot(planeNormals[3]);

		boxPoints[0] = m_position + rotExtents;
		boxPoints[1] = m_position + Vector2(-rotExtents.x, rotExtents.y);
		boxPoints[2] = m_position - rotExtents;
		boxPoints[3] = m_position + Vector2(rotExtents.x, -rotExtents.y);
	}
	break;
	}
	//Clipping
	for (int i = 0; i < 3; i++) {
		//Clip against four planes with sutherland hodgmann, only ignoring those sitting in reference face
		//If pt one is out and next in, Record intersection point and point in. If both are in, record second point.
		//If pt one is in and pt two out record intersection point
		//In our case, ignore all intersection point
		Vector2 pt = boxPoints[i];
		Vector2 nextPt = boxPoints[i + 1];
		bool ptIn = true;
		bool nextPtIn = true;
		for (int j = 0; j < 4; j++)
		{
			Vector2 curPlaneNormal = planeNormals[j];
			decimal curPlaneDist = planeDists[j];
			if (DistPtToPlane(pt, curPlaneNormal, curPlaneDist) > 0) {
				ptIn = false;
			}
			if (DistPtToPlane(nextPt, curPlaneNormal, curPlaneDist) > 0) {
				nextPtIn = false;
			}
		}

		if (!ptIn)
		{
			if (nextPtIn) {
				manifold.contactPoints[manifold.numContactPoints] = nextPt;
				manifold.numContactPoints++;
			}
		}
		else
		{
			if (nextPtIn) {
				manifold.contactPoints[manifold.numContactPoints] = nextPt;
				manifold.numContactPoints++;
			}
		}
	}
	manifold.rb1 = this;
	manifold.rb2 = rb2;
	manifold.normal = minAxis.Dot(aToB) > 0 ? -minAxis : minAxis;
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
		if (pos1Axis + max < pos2Axis + min2)
		{
			//Separating axis
			return false;
		}
		else {
			//Retrieve important info, like penetration amount on such axis
			penetration = pos1Axis + max - (pos2Axis + min2);
			return true;
		}
	}
	else 
	{
		if (pos2Axis + max2 < pos1Axis + min)
		{
			//Separating axis
			return false;
		}
		else {
			penetration = pos2Axis + max2 - (pos1Axis + min);
			return true;
		}
	}
}
