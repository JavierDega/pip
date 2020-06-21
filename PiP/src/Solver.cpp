#include "Solver.h"

#include <iostream>

using namespace std;
using namespace math;

Solver::Solver()
	: m_continuousCollision(false), m_stepMode(false), m_stepOnce(false), m_ignoreSeparatingBodies(true), m_staticResolution(false), m_logCollisionInfo(false),
	m_accumulator(0.f), m_timestep(0.02f), m_gravity(9.8f)
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
	while (dt >  0.f) {
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
	//Integration
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		//Semi euler integration
		Rigidbody* rb = m_rigidbodies[i];
		rb->m_position += rb->m_velocity * dt;
		rb->m_rotation += rb->m_angularVelocity * dt;
		if (!rb->m_isKinematic) rb->m_velocity += rb->m_acceleration * dt;
		rb->m_acceleration = Vector2(0, -m_gravity / rb->m_mass);
		//Send dynamic bodies to sleep
		if (!rb->m_isKinematic) {
			if (rb->m_velocity.LengthSqr() < FLT_EPSILON) {
				rb->m_timeInSleep += dt;
				//If its static for two timesteps or more, put to sleep
				if (rb->m_timeInSleep >= m_timestep * 2) {
					rb->m_isSleeping = true;
				}
			}
			else {
				if (rb->m_isSleeping) {
					rb->m_isSleeping = false;
				}
				if (rb->m_timeInSleep > 0) {
					rb->m_timeInSleep = 0;
				}
			}
		}
	}
}

void Solver::ComputeResponse(const Manifold& manifold)
{

	Rigidbody* rb1 = manifold.rb1;
	Rigidbody* rb2 = manifold.rb2;
	//Collision normal point to A by convention
	Vector2 n = manifold.normal;//Expected to come normalized already
	decimal pen = manifold.penetration;
	decimal invMassA = rb1->m_isKinematic ? 0 : 1 / rb1->m_mass;
	decimal invMassB = rb2->m_isKinematic ? 0 : 1 / rb2->m_mass;
	decimal invIA = rb1->m_isKinematic ? 0 : 1 / rb1->m_inertia;//Inverse Inertia
	decimal invIB = rb2->m_isKinematic ? 0 : 1 / rb2->m_inertia;
	//Make multiple contact points work by caching velocities and then summing local impulses
	Vector2 resultVelA = rb1->m_velocity;
	Vector2 resultVelB = rb2->m_velocity;
	decimal resultAngVelA = rb1->m_angularVelocity;
	decimal resultAngVelB = rb2->m_angularVelocity;
	decimal e = Sqrt(rb1->m_e * rb2->m_e); //Coefficient of restitution
	Vector2 avgContactPoint = Vector2();
	//#TODO: Not final
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
	Vector2 vba = va - vb;//#There might be a problem with this collision response approach for objects that dont directly strike each other
	decimal vbaDotN = vba.Dot(n);
	if (m_staticResolution) {
		//Generic solution that uses manifold's penetration to displace rigidbodies along the normal
		//If we do this, will kinematic objects get displaced by much?
		decimal dispFactor = (rb1->m_isKinematic) ? 0 : (rb2->m_isKinematic) ? 1 : 0.5;
		rb1->m_position += pen * n * dispFactor;
		rb2->m_position -= pen * n * (1 - dispFactor);
	}
	if (m_ignoreSeparatingBodies) {
		if (vbaDotN > 0) return;//Possibly log this, helps solve interpenetration after response, by ignoring separating bodies
	}
	_ASSERT(vbaDotN < 0);
	decimal num = -(1 + e) * vbaDotN;
	decimal denom = invMassA + invMassB + Pow(raP.Dot(n), 2) * invIA + Pow(rbP.Dot(n), 2) * invIB;
	decimal impulse = num / denom;

	_ASSERT(impulse > 0);
	//impulse = Abs(impulse);
	resultVelA += impulse * n * invMassA;
	resultAngVelA += raP.Dot(impulse * n) * invIA;
	resultVelB -= impulse * n * invMassB;
	resultAngVelB -= rbP.Dot(impulse * n) * invIB;

	if (m_logCollisionInfo) {
		cout << "-----------------------------------Collision Response Info-------------------------------" << endl
			<< "Normal: " << n << endl
			<< "ra (rb1 to contact point): " << ra << endl
			<< "rb (rb2 to contact point): " << rb << endl
			<< "rb1 velocity: " << rb1->m_velocity << " velocity at contact point: " << va << endl
			<< "rb2 velocity: " << rb2->m_velocity << " velocity at contact point: " << vb << endl
			<< "relative velocity vba: " << vba << "combined coefficient of restituion (e): " << e << endl
			<< "impulse = " << num << " / " << denom << " = " << impulse << endl
			<< "Result: velA: x(" << resultVelA << " velB = " << resultVelB << endl
			<< "angVelA = " << resultAngVelA << " angVelB = " << resultAngVelB << endl;
	}

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
//Go through custom allocator
Circle* Solver::CreateCircle(decimal rad, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	return nullptr;
}

Capsule* Solver::CreateCapsule(decimal length, decimal radius, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	return nullptr;
}

OrientedBox* Solver::CreateOrientedBox(math::Vector2 halfExtents, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	return nullptr;
}
