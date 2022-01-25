#include "OrientedBox.h"

#include "Circle.h"
#include "Capsule.h"

using namespace PipMath;

OrientedBox::OrientedBox(Vector2 halfExtents, Vector2 pos, decimal rot, Vector2 vel, decimal angVel, decimal mass,
	decimal e, bool isKinematic, decimal kFriction)
	: Rigidbody(pos, rot, vel, angVel, mass, e, isKinematic, kFriction), m_halfExtents(halfExtents)
{
	m_bodyType = BodyType::Obb;
	//Find inertia tensor formula for an oriented box (Derived from capsule's)
	m_inertia = m_mass * (Pow(m_halfExtents.x * 2, 2) + Pow(m_halfExtents.y * 2, 2)) / 12;
}

OrientedBox::~OrientedBox()
{
}
//Intersect AABB for Quad Nodes (Simplified SAT?)
bool OrientedBox::IntersectWith(Vector2 topRight, Vector2 bottomLeft)
{
	Vector2 points[4]{ m_halfExtents, -m_halfExtents,
	Vector2(m_halfExtents.x, -m_halfExtents.y), Vector2(-m_halfExtents.x, m_halfExtents.y) };

	Vector2 points2[4]{ topRight, bottomLeft,
	Vector2(topRight.x, bottomLeft.y), Vector2(bottomLeft.x, topRight.y) };

	//Rotate points to get real positions according to OBB rotation
	for (int i = 0; i < 4; i++)
	{
		points[i].Rotate(m_rotation);
		//points2[i].Rotate(rb2->m_rotation);
	}

	Vector2 quadCenter = topRight + (bottomLeft - topRight) / 2;
	decimal dummyPenetration;
	Vector2 axis = Vector2(1, 0);
	if (!TestAxis(axis, m_position, quadCenter, points, points2, dummyPenetration)) return false;
	axis = Vector2(0, 1);
	if (!TestAxis(axis, m_position, quadCenter, points, points2, dummyPenetration)) return false;
	axis = Vector2(1, 0).Rotate(m_rotation);
	if (!TestAxis(axis, m_position, quadCenter, points, points2, dummyPenetration)) return false;
	axis = Vector2(0, 1).Rotate(m_rotation);
	if (!TestAxis(axis, m_position, quadCenter, points, points2, dummyPenetration)) return false;
	return true;
}

bool OrientedBox::IntersectWith(Rigidbody* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Circle* rb2, Manifold& manifold)
{
	//ClosestPtCircleToObb query
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(Capsule* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool OrientedBox::IntersectWith(OrientedBox* rb2, Manifold& manifold)
{
	//SAT
	//We only have 4 axis to project to, but we can simplify it by bringing things to one Obb's reference frame
	Vector2 aToB = rb2->m_position - m_position;
	//Vector2 rotExtents = m_halfExtents.Rotated(m_rotation);
	//Vector2 rotExtents2 = rb2->m_halfExtents.Rotated(rb2->m_rotation);
	
	//Points in clockwise order
	Vector2 points[4]{ m_halfExtents, Vector2(m_halfExtents.x, -m_halfExtents.y),
	 -m_halfExtents, Vector2(-m_halfExtents.x, m_halfExtents.y) };

	Vector2 points2[4]{ rb2->m_halfExtents, Vector2(rb2->m_halfExtents.x, -rb2->m_halfExtents.y),
	 -rb2->m_halfExtents, Vector2(-rb2->m_halfExtents.x, rb2->m_halfExtents.y) };
	
	//Rotate points to get real positions according to OBB's rotations
	for (int i = 0; i < 4; i++)
	{
		points[i].Rotate(m_rotation);
		points2[i].Rotate(rb2->m_rotation);
	}
	//Possibly add ref arguments to retrieve contact data (amount of penetration,..)
	decimal minPen;
	Vector2 minAxis;
	Vector2 axis;
	decimal penetration;
	SatCollision collisionType;
	//rb1's axii
	axis = Vector2(1, 0).Rotate(m_rotation);
	if (TestAxis(axis, m_position, rb2->m_position, points, points2, penetration)) {
		//Store penetration and axis
		minPen = penetration;
		minAxis = axis;
		collisionType = SatCollision::OBJ1;
	}
	else return false;
	axis = Vector2(0, 1).Rotate(m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, points, points2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ1;
		}
	} 
	else return false;
	//rb2's axii
	axis = Vector2(1, 0).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, points, points2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2;
		}
	} 
	else return false;
	axis = Vector2(0, 1).Rotate(rb2->m_rotation);
	if (TestAxis( axis, m_position, rb2->m_position, points, points2, penetration)) {
		if (penetration < minPen) {
			minPen = penetration;
			minAxis = axis;
			collisionType = SatCollision::OBJ2;
		}
	}
	else return false;

	//#Contact retrieval
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
		
		planeDists[0] = (m_position + m_halfExtents.x * planeNormals[0]).Dot(planeNormals[0]);
		planeDists[1] = (m_position + m_halfExtents.x * planeNormals[1]).Dot(planeNormals[1]);
		planeDists[2] = (m_position + m_halfExtents.y * planeNormals[2]).Dot(planeNormals[2]);
		planeDists[3] = (m_position + m_halfExtents.y * planeNormals[3]).Dot(planeNormals[3]);
		//Points to clip (Need to be in order for clipping to work!)
		boxPoints[0] = rb2->m_position + points2[0];
		boxPoints[1] = rb2->m_position + points2[1];
		boxPoints[2] = rb2->m_position + points2[2];
		boxPoints[3] = rb2->m_position + points2[3];
	}
	break;
	case SatCollision::OBJ2:
	{
		planeNormals[0] = Vector2(1, 0).Rotate(rb2->m_rotation);
		planeNormals[1] = -planeNormals[0];
		planeNormals[2] = Vector2(0, 1).Rotate(rb2->m_rotation);
		planeNormals[3] = -planeNormals[2];

		planeDists[0] = (rb2->m_position + rb2->m_halfExtents.x * planeNormals[0]).Dot(planeNormals[0]);
		planeDists[1] = (rb2->m_position + rb2->m_halfExtents.x * planeNormals[1]).Dot(planeNormals[1]);
		planeDists[2] = (rb2->m_position + rb2->m_halfExtents.y * planeNormals[2]).Dot(planeNormals[2]);
		planeDists[3] = (rb2->m_position + rb2->m_halfExtents.y * planeNormals[3]).Dot(planeNormals[3]);

		boxPoints[0] = m_position + points[0];
		boxPoints[1] = m_position + points[1];
		boxPoints[2] = m_position + points[2];
		boxPoints[3] = m_position + points[3];
	}
	break;
	}
	//Clipping
	for (int i = 0; i < 4; i++) {
		//Clip against four planes with sutherland hodgmann, only ignoring those sitting in reference face
		//If pt one is out and next in, Record intersection point and point in. If both are in, record second point.
		//If pt one is in and pt two out record intersection point
		//In our case, ignore all intersection point
		Vector2 pt = boxPoints[i];
		Vector2 nextPt = boxPoints[ (i + 1 < 4) ? i + 1 : 0];
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
	manifold.penetration = minPen;
	return true;
}

decimal OrientedBox::SweepWith(Rigidbody* rb2, decimal dt, Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal OrientedBox::SweepWith(Circle* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(Capsule* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal OrientedBox::SweepWith(OrientedBox* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

bool OrientedBox::TestAxis(Vector2 axis, Vector2 pos1, Vector2 pos2, Vector2 points[], Vector2 points2[], decimal& penetration)
{
	decimal pos1Axis = pos1.Dot(axis);
	decimal pos2Axis = pos2.Dot(axis);
	decimal min = 0, max = 0;
	decimal min2 = 0, max2 = 0;

	for (int i = 0; i < 4; i++) {
		decimal projection = points[i].Dot(axis);
		decimal projection2 = points2[i].Dot(axis);
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
