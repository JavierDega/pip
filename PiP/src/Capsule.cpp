
#include "Capsule.h"
#include "Circle.h"
#include "OrientedBox.h"

using namespace math;

Capsule::Capsule(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, bool isKinematic, 
	decimal length, decimal radius)
	: Rigidbody(pos, rot, vel, angVel, accel, mass, isKinematic), m_length(length), m_radius(radius)
{
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
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * m_radius;
		Vector2 sphereEdge = c + closestVec * rb2->m_radius;
		manifold.normal = closestVec;//Point to A by convention
		manifold.numContactPoints = 1;
		manifold.contactPoints[0] = (capsuleEdge + sphereEdge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	closestPt = ClosestPtToSegment(a, b, d);
	closestVec = closestPt - d;
	if (closestVec.LengthSqr() <= rab * rab) {
		//Fill manifold
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * m_radius;
		Vector2 sphereEdge = d + closestVec * rb2->m_radius;
		manifold.normal = closestVec;//Point to A
		manifold.numContactPoints = 1;
		manifold.contactPoints[0] = (capsuleEdge + sphereEdge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	closestPt = ClosestPtToSegment(c, d, a);//Segment in caps2, to point in caps1
	closestVec = closestPt - a;
	if (closestVec.LengthSqr() <= rab*rab) {
		//Fill manifold
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = a + closestVec * m_radius;
		manifold.normal = -closestVec;//Point to A
		manifold.numContactPoints = 1;
		manifold.contactPoints[0] = (capsuleEdge + sphereEdge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	closestPt = ClosestPtToSegment(c, d, b);
	closestVec = closestPt - b;
	if (closestVec.LengthSqr() <= rab * rab) {
		//Fill manifold
		Vector2 capsuleEdge = closestPt - closestVec.Normalize() * rb2->m_radius;
		Vector2 sphereEdge = b + closestVec * m_radius;
		manifold.normal = -closestVec;//Point to A
		manifold.numContactPoints = 1;
		manifold.contactPoints[0] = (capsuleEdge + sphereEdge) / 2;
		manifold.rb1 = this;
		manifold.rb2 = rb2;
		return true;
	}
	return false;
}

bool Capsule::IntersectWith(OrientedBox* rb2, math::Manifold& manifold)
{
	//Haven't done this one before, probably ClosestPtObbToSegment query
	///#Randy Gaul: 
	//First make sure the AABB is on one side of the plane or the other.
	//Then compute the support point of the AABB along the normal of the plane,
	//and clamp this point within your lane segment.

	/*Vector2 n = Vector2(0, 1).Rotate(m_rotation);
	decimal dist = m_position.Dot(n);
	Vector2 boxPoints[4];
	Vector2 rotExtents = rb2->m_halfExtents.Rotated(rb2->m_rotation);
	boxPoints[0] = rb2->m_position + rotExtents;
	boxPoints[1] = rb2->m_position + rotExtents.Perp();
	boxPoints[2] = rb2->m_position - rotExtents;
	boxPoints[3] = rb2->m_position - rotExtents.Perp();
	if (DistPtToPlane(rb2->m_position, n, dist) > 0) {
		//On positive side of plane formed by the capsule
		//Find those close enough to the segment (Dist = radius), and clamped to the capsule's segment
		for (int i = 0; i < 4; i++) {
			Vector2 curBoxPoint = boxPoints[i];
			if (DistPtToPlane(curBoxPoint, n, dist) < m_radius) 
			{
				Vector2 a = m_position - Vector2(1, 0).Rotate(m_rotation) * m_length / 2;
				Vector2 b = a + Vector2(1, 0).Rotate(m_rotation) * m_length;
				Vector2 ab = b - a;
				Vector2 ac = curBoxPoint - a;
					//Clamp to line segment.
					//Could be in line segment (dot AB.AC > 0 && dot BA.BC > 0. Or could be sphere check against capsule edges)
				if (ab.Dot(ac) < 0) {
					if (ac.LengthSqr() > m_radius * m_radius) {
						//This point does not collide with the capsule
						break;
					}
				}
				if (-ab.Dot(ac) < 0) {
					if (ac.LengthSqr() > m_radius* m_radius) {
						//This point does not collide with a circle in the capsule
						break;
					}
				}
			}
		}
	}
	else {
		//On negative side of the plane
	}*/

	///#Some coder:
	//use the formula for the distance from a point to a line in 2d, do it for all four corners.
	//the lowest result is the closest corner (or a corner on the closest side, if the line happens to be axis aligned).
	
	//Rotate to get line-AABB query
	//Rotate by cap's rotation and - rotation of the Obb to get it in the AABB's reference frame

	//Get Capsule's AB
	decimal halfLength = m_length / 2;
	Vector2 a = Vector2{ -halfLength, 0 };
	//Rotate about position
	a.Rotate(m_rotation - rb2->m_rotation);
	Vector2 b = -a;
	a += m_position - rb2->m_position;
	b += m_position - rb2->m_position;
	//Caps points are now in box's local space
	Vector2 boxPoints[4] = {
		rb2->m_halfExtents,
		rb2->m_halfExtents.Perp(),
		-rb2->m_halfExtents,
		-rb2->m_halfExtents.Perp()
	};
	
	for (int i = 0; i < 3; i++) {
		//Generate box segment and do like Caps-Caps query
		Vector2 c = boxPoints[i];
		Vector2 d = boxPoints[(i < 3) ? i + 1 : 0];

		Vector2 closestPt = ClosestPtToSegment(a, b, c);//Segment in caps1, to point in caps2
		Vector2 closestVec = closestPt - c;//Point to closestpt in caps segment, also normal
		if (closestVec.LengthSqr() <= m_radius * m_radius) {
			//Fill manifold
			manifold.normal = closestVec;//Point to A by convention
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = c;
			manifold.rb1 = this;
			manifold.rb2 = rb2;
			return true;
		}
		closestPt = ClosestPtToSegment(a, b, d);
		closestVec = closestPt - d;
		if (closestVec.LengthSqr() <= m_radius * m_radius) {
			//Fill manifold
			manifold.normal = closestVec;//Point to A
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = d;
			manifold.rb1 = this;
			manifold.rb2 = rb2;
			return true;
		}
		closestPt = ClosestPtToSegment(c, d, a);//Segment in caps2, to point in caps1
		closestVec = closestPt - a;
		if (closestVec.LengthSqr() <= m_radius * m_radius) {
			//Fill manifold
			manifold.normal = -closestVec;//Point to A
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = closestPt;
			manifold.rb1 = this;
			manifold.rb2 = rb2;
			return true;
		}
		closestPt = ClosestPtToSegment(c, d, b);
		closestVec = closestPt - b;
		if (closestVec.LengthSqr() <= m_radius * m_radius) {
			//Fill manifold
			manifold.normal = -closestVec;//Point to A
			manifold.numContactPoints = 1;
			manifold.contactPoints[0] = closestPt;
			manifold.rb1 = this;
			manifold.rb2 = rb2;
			return true;
		}
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

decimal Capsule::SweepWith(OrientedBox* rb2, decimal dt, math::Manifold& manifold)
{
	return decimal();
}
