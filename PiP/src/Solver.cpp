#include "Solver.h"

#include <iostream>

using namespace std;
using namespace math;

Solver::Solver()
	: m_continuousCollision(false), m_stepMode(false), m_stepOnce(false), m_ignoreSeparatingBodies(true), m_staticResolution(false), m_accumulator(0.f), m_timestep(0.02f), m_gravity(9.8f)
{
}


Solver::~Solver()
{
	m_rigidbodies.clear();
}

void Solver::Update(decimal dt)
{
	//Step through mem allocated bodies

	//Fixed timestep with accumulator (50fps)
	if (m_stepMode) {
		if (m_stepOnce) {
			(m_continuousCollision) ? ContinuousStep(m_timestep) : Step(m_timestep);
			m_stepOnce = false;
		}
	}
	else {
		m_accumulator += dt;
		if (m_accumulator > 0.2f) m_accumulator = 0.2f;
		while (m_accumulator > m_timestep) {
			(m_continuousCollision) ? ContinuousStep(m_timestep) : Step(m_timestep);
			m_accumulator -= m_timestep;
		}
	}
	//@UP TO THE GRAPHICS APPLICATION:
	//To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	//float alpha = m_accumulator / m_timestep;
}

void Solver::ContinuousStep(decimal dt) 
{
	//We need to have up to date velocities to perform sweeps
	//1: Perform sweeps, storing collision deltas
	//2: Only acknowledge first collision time
	//3: Step everything upto first collision time
	//4: Compute responses for first colliding pair, everything else retains velocity
	//5: Reperform sweeps  (for(i){for (j = i + 1; ..)}
	//6: If new collisions, repeat
	//7: Apply gravity forces (Collisions are impulse based, friction is force based) *optionally do this at step 0
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		//Semi euler integration
		Rigidbody* rb = m_rigidbodies[i];
		rb->m_acceleration = Vector2(0, -m_gravity / rb->m_mass);
		if (!rb->m_isKinematic) rb->m_velocity += rb->m_acceleration*dt;
	}
	while ( dt > (decimal) 0.f) {
		decimal firstCollision = 1;//Normalized dt
		Manifold firstManifold = Manifold();
		//Upwards collision check
		for (int i = 0; i < m_rigidbodies.size(); i++) {
			for (int j = i + 1; j < m_rigidbodies.size(); j++) {
				Manifold currentManifold = Manifold();
				if (decimal t = m_rigidbodies[i]->SweepWith(m_rigidbodies[j], dt, currentManifold)) {
					//They collide during the frame, store
					if ( t <= firstCollision) {
						firstCollision = t;
						firstManifold = currentManifold;
					}
				}
			}
		}
		//Step everything upto collision time
		decimal timeToStep = dt * firstCollision;
		for (int i = 0; i < m_rigidbodies.size(); i++) {
			//Step according to their velocities (Don't apply gravity or frictional accelerations).
			Rigidbody * rb = m_rigidbodies[i];
			rb->m_position += rb->m_velocity*timeToStep;
			rb->m_rotation += rb->m_angularVelocity*timeToStep;
		}
		dt -= timeToStep;
		//If there was collision
		if (firstManifold.rb1) {
			ComputeResponse(firstManifold);
		}
	}
}

void Solver::Step(decimal dt)
{
	//Integration
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		//Semi euler integration
		Rigidbody* rb = m_rigidbodies[i];
		rb->m_acceleration = Vector2(0, -m_gravity / rb->m_mass);
		if(!rb->m_isKinematic) rb->m_velocity += rb->m_acceleration * dt;
		rb->m_position += rb->m_velocity * dt;
		rb->m_rotation += rb->m_angularVelocity * dt;
	}
	//Upwards collision check
	m_currentManifolds.clear();
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		for (int j = i + 1; j < m_rigidbodies.size(); j++) {
			Manifold currentManifold;
			if (m_rigidbodies[i]->IntersectWith(m_rigidbodies[j], currentManifold)) {
				//They collide during the frame, store
				m_currentManifolds.push_back(currentManifold);//add manifolds
			}
		}
	}
	//Collision response
	for (Manifold manifold : m_currentManifolds) ComputeResponse(manifold);

}

void Solver::ComputeResponse(const Manifold& manifold)
{
	/*// First, find the normalized vector n from the center of
	// circle1 to the center of circle2
	Vector n = circle1.center - circle2.center;
	n.normalize();
	// Find the length of the component of each of the movement
	// vectors along n.
	// a1 = v1 . n
	// a2 = v2 . n
	float a1 = v1.dot(n);
	float a2 = v2.dot(n);

	// Using the optimized version,
	// optimizedP =  2(a1 - a2)
	//              -----------
	//                m1 + m2
	float optimizedP = (2.0 * (a1 - a2)) / (circle1.mass + circle2.mass);

	// Calculate v1', the new movement vector of circle1
	// v1' = v1 - optimizedP * m2 * n
	Vector v1' = v1 - optimizedP * circle2.mass * n;

	// Calculate v1', the new movement vector of circle1
	// v2' = v2 + optimizedP * m1 * n
	Vector v2' = v2 + optimizedP * circle1.mass * n;

	circle1.setMovementVector(v1');
	circle2.setMovementVector(v2');
	// Find the length of the component of each of the movement vectors along n.
	//decimal a1 = rb1->m_velocity.Dot(n);
	//decimal a2 = rb2->m_velocity.Dot(n);
	// Using the optimized version,
	// optimizedP =  2(a1 - a2)
	//              -----------
	//                m1 + m2
	//decimal optimizedP = ((decimal)2.f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);
	// Calculate v1', the new movement vector of circle1
	// v1' = v1 - optimizedP * m2 * n
	//rb1->m_velocity -= n * optimizedP * rb2->m_mass;
	// Calculate v1', the new movement vector of circle1
	// v2' = v2 + optimizedP * m1 * n
	//rb2->m_velocity += n * optimizedP * rb1->m_mass;

	//Use contact points for rotation
	//torque = Force X (Contact point - center of mass);
	//We want to apply angular impulse ie an immediate change in angular velocity
	//Use Vector2Str.Cross>*/

	//Chris Hecker's physics column (Using j impulse)
	//Norma expected to point to A, e = coefficient of restitution (0 = inelastic, 1 = elastic)
	//ra = perp of A to point of contact, same rb 
	//j = -((1 + e)*vAB * n)/( n*n*(1/ma + 1/mb) + (ra*n)^2/Ia + (rb*n)^2/Ib )
	//va' = v1 + j*n/ma 
	//vb' = v2 - j*n/mb
	//wa' = wa + r1*j*n/Ia;
	//wb' = wb - r2*j*n/Ib;

	//Chris Hecker's physics sample
	/*vector_2 Position =
		Configuration.BoundingBox.aVertices[CollidingCornerIndex];

	vector_2 CMToCornerPerp = GetPerpendicular(Position -
		Configuration.CMPosition);

	vector_2 Velocity = Configuration.CMVelocity +
		Configuration.AngularVelocity * CMToCornerPerp;

	real ImpulseNumerator = -(r(1) + Body.CoefficientOfRestitution) *
		DotProduct(Velocity, CollisionNormal);

	float PerpDot = DotProduct(CMToCornerPerp, CollisionNormal);

	real ImpulseDenominator = Body.OneOverMass +
		Body.OneOverCMMomentOfInertia * PerpDot * PerpDot;

	real Impulse = ImpulseNumerator / ImpulseDenominator;

	Configuration.CMVelocity += Impulse * Body.OneOverMass * CollisionNormal;

	Configuration.AngularVelocity +=
		Impulse * Body.OneOverCMMomentOfInertia * PerpDot;
		*/

	Rigidbody* rb1 = manifold.rb1;
	Rigidbody* rb2 = manifold.rb2;
	//Collision normal point to A by convention
	Vector2 n = manifold.normal;//Expected to come normalized already
	decimal ma = rb1->m_mass;
	decimal mb = rb2->m_mass;
	decimal ia = rb1->m_inertia;
	decimal ib = rb2->m_inertia;
	//Make multiple contact points work by caching velocities and then summing local impulses
	Vector2 resultVelA = rb1->m_velocity;
	Vector2 resultVelB = rb2->m_velocity;
	decimal resultAngVelA = rb1->m_angularVelocity;
	decimal resultAngVelB = rb2->m_angularVelocity;
	decimal e = 1; //Coefficient of restitution
	Vector2 avgContactPoint = Vector2();
	for (int i = 0; i < manifold.numContactPoints; i++) 
	{
		avgContactPoint += manifold.contactPoints[i];
	}
	avgContactPoint /= manifold.numContactPoints;

	Vector2 ra = (avgContactPoint - rb1->m_position);
	Vector2 rb = (avgContactPoint - rb2->m_position);
	Vector2 raP = ra.Perp();
	Vector2 rbP = rb.Perp();
	//Velocity at contact point seems not to change even with rotating bodies
	Vector2 va = rb1->m_velocity + rb1->m_angularVelocity * raP;
	Vector2 vb = rb2->m_velocity + rb2->m_angularVelocity * rbP;
	Vector2 vba = va - vb;// There might be a problem with this collision response approach for objects that dont directly strike each other
	decimal vbaDotN = vba.Dot(n);
	if (m_ignoreSeparatingBodies) {
		if (vbaDotN > 0) return;//Possibly log this, helps solve interpenetration after response, by ignoring separating bodies
	}
	if (m_staticResolution) {
		//Generic solution that uses manifold's penetration to displace rigidbodies along the normal

	}
	_ASSERT(vbaDotN < 0);
	decimal num = -(1 + e) * vbaDotN;
	//decimal denom = 1 / ma + 1 / mb + (ra3d.Cross(ra3d.Cross(n3d)) / ia + rb3d.Cross(rb3d.Cross(n3d)) / ib).Dot(n3d);
	decimal denom = 1 / ma + 1 / mb + Pow(raP.Dot(n), 2) / ia + Pow(rbP.Dot(n), 2) / ib;
	decimal impulse = num / denom;
	cout << "-----------------------------------Collision Response Info-------------------------------" << endl;
	cout << "Normal: x(" << n.x << ") y(" << n.y << ")" << endl;
	cout << "ra (rb1 to contact point): x(" << ra.x << ") y(" << ra.y << ")" << endl;
	cout << "rb (rb2 to contact point): x(" << rb.x << ") y(" << rb.y << ")" << endl;
	cout << "rb1 velocity: x(" << rb1->m_velocity.x << ") y(" << rb1->m_velocity.y << ") velocity at contact point: x(" <<
		va.x << ") y(" << va.y << ")" << endl;
	cout << "rb2 velocity: x(" << rb2->m_velocity.x << ") y(" << rb2->m_velocity.y << ") velocity at contact point: x(" <<
		vb.x << ") y(" << vb.y << ")" << endl;
	cout << "relative velocity vba:" << "x(" << vba.x << ") y(" << vba.y << ")" << endl;
	cout << "impulse = " << num << " / " << denom << " = " << impulse << endl;
	_ASSERT(impulse > 0);
	impulse = Abs(impulse);
	resultVelA += impulse * n / rb1->m_mass;
	//resultAngVelA += impulse * ra3d.Cross(n3d).z / ia;
	resultAngVelA += raP.Dot(impulse * n) / ia;
	resultVelB -= impulse * n / rb2->m_mass;
	//resultAngVelB -= impulse * rb3d.Cross(n3d).z / ib;
	resultAngVelB -= rbP.Dot(impulse * n) / ib;

	rb1->m_velocity = resultVelA;
	rb1->m_angularVelocity = resultAngVelA;
	rb2->m_velocity = resultVelB;
	rb2->m_angularVelocity = resultAngVelB;
}

Rigidbody * Solver::AddBody(Rigidbody * rb)
{
	//Create default sphere and pass reference, using allocator
	m_rigidbodies.push_back(rb);
	return m_rigidbodies.back();
}
