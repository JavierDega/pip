#include "Capsule.h"

#include <float.h>

#include "Circle.h"
#include "OrientedBox.h"

using namespace PipMath;

Capsule::Capsule(decimal length, decimal radius, Vector2 pos, decimal rot, Vector2 vel, decimal angVel, decimal mass, decimal e,
 bool isKinematic, decimal kFriction)
	: m_length(length), m_radius(radius), Rigidbody(pos, rot, vel, angVel, mass, e, isKinematic, kFriction)
{
	m_bodyType = BodyType::Capsule;
	//Inertia tensor of two hemispheres (Parallel axis to translate CM) + the one of the rectangle
	//Calculate mass of individual parts (assuming uniform density)
	decimal areaHemicircles = m_radius * m_radius * PI;
	decimal areaRectangle = m_length * m_radius * 2;
	decimal area = areaHemicircles + areaRectangle;
	decimal massHemicircles = m_mass * areaHemicircles / area;
	decimal massRectangle = m_mass - massHemicircles;
	decimal inertiaRectangle = massRectangle * (Pow(m_radius * 2, 2) + m_length * m_length) / 12;
	//Use parallel axis to add hemispheres inertia according to capsule CM
	//I = Icm + md^2
	decimal inertiaCircle1 = massHemicircles * m_radius * m_radius / 2;
	decimal inertiaCircle2 = inertiaCircle1 - massHemicircles * m_length / 2;
	m_inertia = inertiaRectangle + inertiaCircle2;
}

Capsule::~Capsule()
{
}
//Intersect test with AABB (Quad Nodes)
bool Capsule::IntersectWith(Vector2 topRight, Vector2 bottomLeft)
{
	//#Early out tests
	//Equation of a line
	//Caps line	y = mx + c
	decimal m = Tan(m_rotation);
	//Clamp line segment to aabb, compare sqdist to sqrRad
	//Get Capsule's AB
	decimal halfLength = m_length / 2;
	Vector2 a = Vector2{ -halfLength, 0 };
	Vector2 b = Vector2{ halfLength, 0 };
	//Rotate about position
	a.Rotate(m_rotation);
	b.Rotate(m_rotation);
	a += m_position;
	b += m_position;
	decimal c = a.y - m*a.x;

	Vector2 boxPoints[4] =
	{
		topRight,
		Vector2(topRight.x, bottomLeft.y), //bottomRight
		bottomLeft,
		Vector2(bottomLeft.x, topRight.y) //topLeft
	};

	//First, clamp both points in caps to aabb and check if theyre close enough
	Vector2 aClamped = Vector2(Clamp(a.x, bottomLeft.x, topRight.x), Clamp(a.y, bottomLeft.y, topRight.y));
	Vector2 bClamped = Vector2(Clamp(b.x, bottomLeft.x, topRight.x), Clamp(b.y, bottomLeft.y, topRight.y));
	if ((aClamped - a).LengthSqr() <= m_radius * m_radius) return true;
	if ((bClamped - b).LengthSqr() <= m_radius * m_radius) return true;
	//Then if none are close enough check if capsule line collides with aabb axis, and then check distance against points clamped in segment and in aabb
	//First check line is not parallel to y or x axis
	decimal segmentXMin, segmentXMax, segmentYMin, segmentYMax;
	if (a.x <= b.x)
	{
		segmentXMin = a.x;
		segmentXMax = b.x;
	}
	else
	{
		segmentXMin = b.x;
		segmentXMax = a.x;
	}
	if (a.y <= b.y)
	{
		segmentYMin = a.y;
		segmentYMax = b.y;
	}
	else
	{
		segmentYMin = b.y;
		segmentYMax = a.y;
	}

	if (m == 0) {
		//Completely horizontal line segment
		//Find points when it collides with the x min and max of the aabb
		//Clamp those to the line segment, and to the aabb, and compare

		decimal y = a.y;
		decimal x = bottomLeft.x;
		decimal x2 = topRight.x;
		decimal boxClampY = Clamp(y, bottomLeft.y, topRight.y);
		Vector2 boxClamp1 = Vector2(bottomLeft.x, boxClampY);
		Vector2 boxClamp2 = Vector2(topRight.x, boxClampY);

		Vector2 segmentClamp1 = Vector2(Clamp(x, segmentXMin, segmentXMax), y);
		Vector2 segmentClamp2 = Vector2(Clamp(x2, segmentXMin, segmentXMax), y);
		
		//Check distances of segment clamps against box clamps
		if ((segmentClamp1 - boxClamp1).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp1 - boxClamp2).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp2 - boxClamp1).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp2 - boxClamp2).LengthSqr() <= m_radius * m_radius) return true;
	}
	else if (m == FLT_MAX) 
	{
		//Completely vertical line segment
		//Find points when it collides with the y min and max of the aabb
		//Clamp those to the line segment, and to the aabb, and compare

		decimal x = a.x;
		decimal y = bottomLeft.y;
		decimal y2 = topRight.y;
		decimal boxClampX = Clamp(x, bottomLeft.x, topRight.x);
		Vector2 boxClamp1 = Vector2(boxClampX, bottomLeft.y);
		Vector2 boxClamp2 = Vector2(boxClampX, topRight.y);

		Vector2 segmentClamp1 = Vector2(x, Clamp(y, segmentYMin, segmentYMax));
		Vector2 segmentClamp2 = Vector2(x, Clamp(y2, segmentYMin, segmentYMax));

		//Check distances of segment clamps against box clamps
		if ((segmentClamp1 - boxClamp1).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp1 - boxClamp2).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp2 - boxClamp1).LengthSqr() <= m_radius * m_radius) return true;
		if ((segmentClamp2 - boxClamp2).LengthSqr() <= m_radius * m_radius) return true;
	}
	else
	{
		//Use equation of line to solve for x == xmin || xmax and || y == ymin || ymax
		//We know it collides with all aabb axis as its not parallel to none
		//Line with box x axis 1
		Vector2 lineX1 = Vector2(bottomLeft.x, m * bottomLeft.x + c);
		Vector2 lineX2 = Vector2(topRight.x, m * topRight.x + c);

		Vector2 lineY1 = Vector2((bottomLeft.y - c) / m, bottomLeft.y);
		Vector2 lineY2 = Vector2((topRight.y - c) / m, topRight.y);

		Vector2 boxClamp1 = Vector2(bottomLeft.x, Clamp(lineX1.y, bottomLeft.y, topRight.y));
		Vector2 segmentClamp1 = Vector2(Clamp(lineX1.x, segmentXMin, segmentXMax), Clamp(lineX1.y, segmentYMin, segmentYMax));
		if ((boxClamp1 - segmentClamp1).LengthSqr() <= m_radius * m_radius) return true;

		Vector2 boxClamp2 = Vector2(topRight.x, Clamp(lineX2.y, bottomLeft.x, topRight.y));
		Vector2 segmentClamp2 = Vector2(Clamp(lineX2.x, segmentXMin, segmentXMax), Clamp(lineX2.y, segmentYMin, segmentYMax));
		if ((boxClamp2 - segmentClamp2).LengthSqr() <= m_radius * m_radius) return true;

		Vector2 boxClamp3 = Vector2(Clamp(lineY1.x, bottomLeft.x, topRight.x), bottomLeft.y);
		Vector2 segmentClamp3 = Vector2(Clamp(lineY1.x, segmentXMin, segmentXMax), Clamp(lineY1.y, segmentYMin, segmentYMax));
		if ((boxClamp3 - segmentClamp3).LengthSqr() <= m_radius * m_radius) return true;

		Vector2 boxClamp4 = Vector2(Clamp(lineY2.x, bottomLeft.x, topRight.x), topRight.y);
		Vector2 segmentClamp4 = Vector2(Clamp(lineY2.x, segmentXMin, segmentXMax), Clamp(lineY2.y, segmentYMin, segmentYMax));
		if ((boxClamp4 - segmentClamp4).LengthSqr() <= m_radius * m_radius) return true;
	}
	//#UNIT TEST THIS STUFF
	return false;
}

bool Capsule::IntersectWith(Rigidbody* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool Capsule::IntersectWith(Circle* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);//Forward to solution in circle
}

bool Capsule::IntersectWith(Capsule* rb2, Manifold& manifold)
{
	//Get Capsule's AB
	decimal halfLength = m_length / 2;
	decimal halfLength2 = rb2->m_length / 2;
	Vector2 a = Vector2{ -halfLength, 0 };
	Vector2 b = Vector2{ halfLength, 0 };
	Vector2 c = Vector2{ -halfLength2, 0 };
	Vector2 d = Vector2{ halfLength2 , 0 };
	//Rotate about position
	a.Rotate(m_rotation);
	b.Rotate(m_rotation);
	c.Rotate(rb2->m_rotation);
	d.Rotate(rb2->m_rotation);
	a += m_position;
	b += m_position;
	c += rb2->m_position;
	d += rb2->m_position;
	decimal rab = m_radius + rb2->m_radius;

	Vector2 closestPt = ClosestPtToSegment(a, b, c);//Segment in caps1, to point in caps2
	Vector2 closestVec = closestPt - c;//Center of sphere to closestpt in caps segment, also normal
	if (closestVec.LengthSqr() <= rab * rab) {
		//Fill manifold
		manifold.penetration = rab - closestVec.Length();
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * m_radius;
		Vector2 sphereEdge = c + closestVec * rb2->m_radius;
		manifold.normal = closestVec;//Point to A by convention
		manifold.contactPoints[manifold.numContactPoints] = (capsuleEdge + sphereEdge) / 2;//#Things like this assume a static collision resolution that displaces both objects equally
		manifold.numContactPoints++;
	}
	closestPt = ClosestPtToSegment(a, b, d);
	closestVec = closestPt - d;
	if (closestVec.LengthSqr() <= rab * rab) {
		//Fill manifold
		manifold.penetration = rab - closestVec.Length();
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * m_radius;
		Vector2 sphereEdge = d + closestVec * rb2->m_radius;
		manifold.normal = closestVec;//Point to A
		manifold.contactPoints[manifold.numContactPoints] = (capsuleEdge + sphereEdge) / 2;
		manifold.numContactPoints++;
	}
	closestPt = ClosestPtToSegment(c, d, a);//Segment in caps2, to point in caps1
	closestVec = closestPt - a;
	if (closestVec.LengthSqr() <= rab*rab) {
		//Fill manifold
		manifold.penetration = rab - closestVec.Length();
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = a + closestVec * m_radius;
		manifold.normal = -closestVec;//Point to A
		manifold.contactPoints[manifold.numContactPoints] = (capsuleEdge + sphereEdge) / 2;
		manifold.numContactPoints++;
	}
	closestPt = ClosestPtToSegment(c, d, b);
	closestVec = closestPt - b;
	if (closestVec.LengthSqr() <= rab * rab) {
		//Fill manifold
		manifold.penetration = rab - closestVec.Length();
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = b + closestVec * m_radius;
		manifold.normal = -closestVec;//Point to A
		manifold.contactPoints[manifold.numContactPoints] = (capsuleEdge + sphereEdge) / 2;
		manifold.numContactPoints++;
	}
	if (manifold.numContactPoints) {
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	else return false;
}

bool Capsule::IntersectWith(OrientedBox* rb2, Manifold& manifold)
{
	//Haven't done this one before, probably ClosestPtObbToSegment query
	///#Randy Gaul: 
	//First make sure the AABB is on one side of the plane or the other.
	//Then compute the support point of the AABB along the normal of the plane,
	//and clamp this point within your lane segment.

	///#Some coder:
	//use the formula for the distance from a point to a line in 2d, do it for all four corners.
	//the lowest result is the closest corner (or a corner on the closest side, if the line happens to be axis aligned).
	
	//Rotate to get line-AABB query
	//Rotate by cap's rotation and - rotation of the Obb to get it in the AABB's reference frame

	//#Still need to deal with multiple contact points in this case, just like Caps-caps, caps may be laying on a box side, or the opposite

	//Get Capsule's AB
	decimal halfLength = m_length / 2;
	Vector2 a = m_position + Vector2( -halfLength, 0 ).Rotate(m_rotation);
	Vector2 b = m_position + Vector2( halfLength, 0 ).Rotate(m_rotation);

	Vector2 rotExtents = rb2->m_halfExtents.Rotated(rb2->m_rotation);
	//Caps points are now in box's local space
	Vector2 boxPoints[4] = {
		rb2->m_position + rotExtents,
		rb2->m_position + rotExtents.Perp(),
		rb2->m_position - rotExtents,
		rb2->m_position - rotExtents.Perp()
	};
	
	decimal biggestPen = 0;
	for (int i = 0; i < 3; i++) {
		//Generate box segment and do like Caps-Caps query
		Vector2 c = boxPoints[i];
		Vector2 d = boxPoints[(i < 3) ? i + 1 : 0];

		Vector2 closestPt = ClosestPtToSegment(a, b, c);//Segment in caps, to point in box
		Vector2 closestVec = closestPt - c;//Point to closestpt in caps segment, also normal
		if ( m_radius * m_radius - closestVec.LengthSqr() >= biggestPen) {
			//Fill manifold
			manifold.normal = closestVec.Normalized();//Point to A by convention
			manifold.penetration = m_radius - closestVec.Length();
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = c;
			biggestPen = m_radius * m_radius - closestVec.LengthSqr();
		}
		closestPt = ClosestPtToSegment(a, b, d);
		closestVec = closestPt - d;
		if (m_radius * m_radius - closestVec.LengthSqr() >= biggestPen) {
			//Fill manifold
			manifold.normal = closestVec.Normalized();
			manifold.penetration = m_radius - closestVec.Length();
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = d;
			biggestPen = m_radius * m_radius - closestVec.LengthSqr();
		}
		closestPt = ClosestPtToSegment(c, d, a);
		closestVec = closestPt - a;
		if (m_radius * m_radius - closestVec.LengthSqr() >= biggestPen) {
			//Fill manifold
			manifold.normal = -closestVec.Normalized();
			manifold.penetration = m_radius - closestVec.Length();
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = closestPt;
			biggestPen = m_radius * m_radius - closestVec.LengthSqr();
		}
		closestPt = ClosestPtToSegment(c, d, b);
		closestVec = closestPt - b;
		if (m_radius * m_radius - closestVec.LengthSqr() >= biggestPen) {
			//Fill manifold
			manifold.normal = -closestVec.Normalized();
			manifold.penetration = m_radius - closestVec.Length();
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = closestPt;
			biggestPen = m_radius * m_radius - closestVec.LengthSqr();
		}
	}
	if (manifold.numContactPoints) {
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	return false;
}

decimal Capsule::SweepWith(Rigidbody* rb2, decimal dt, Manifold& manifold)
{
	return rb2->SweepWith(this, dt, manifold);
}

decimal Capsule::SweepWith(Circle* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal Capsule::SweepWith(Capsule* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}

decimal Capsule::SweepWith(OrientedBox* rb2, decimal dt, Manifold& manifold)
{
	return decimal();
}
