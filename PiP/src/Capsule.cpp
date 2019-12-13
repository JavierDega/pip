
#include "Capsule.h"
#include "Circle.h"

using namespace math;

Capsule::Capsule(math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, math::Vector2 accel, decimal mass, decimal length, decimal radius)
	: Rigidbody(pos, rot, vel, angVel, accel, mass), m_length(length), m_radius(radius)
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

bool Capsule::ComputeIntersect(Rigidbody* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);
}

bool Capsule::IntersectWith(Circle* rb2, Manifold& manifold)
{
	return rb2->IntersectWith(this, manifold);//Forward to solution in circle
}

bool Capsule::IntersectWith(Capsule* rb2, Manifold& manifold)
{
	return false;
}

decimal Capsule::ComputeSweep(Rigidbody* rb2, decimal dt, Manifold& manifold)
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
