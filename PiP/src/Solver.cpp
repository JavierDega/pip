#include "Solver.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"
#include <iostream>
#include <assert.h>

using namespace std;
using namespace math;

Solver::Solver()
	: m_continuousCollision(false), m_stepMode(true), m_stepOnce(false), m_ignoreSeparatingBodies(true), m_staticResolution(true), m_logCollisionInfo(false),
	m_accumulator(0.f), m_timestep(0.02f), m_gravity(9.8f)
{
	//unsigned int size = sizeof(OrientedBox);
	m_allocator.CreatePool(50*sizeof(OrientedBox));//Each orientedbox size in uint = 80.
}


Solver::~Solver()
{
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
	//#TODO: Move code onto pool iterator
	//We need to have up to date velocities to perform sweeps
	//1: Perform sweeps, storing collision deltas
	//2: Only acknowledge first collision time
	//3: Step everything upto first collision time
	//4: Compute responses for first colliding pair, everything else retains velocity
	//5: Reperform sweeps  (for(i){for (j = i + 1; ..)}
	//6: If new collisions, repeat
	//7: Apply gravity forces (Collisions are impulse based, friction is force based) *optionally do this at step 0
	/*for (int i = 0; i < m_rigidbodies.size(); i++) {
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
	}*/
}

void Solver::Step(decimal dt)
{
	//Upwards collision check
	m_currentManifolds.clear();
	//Integration
	std::vector<Rigidbody*> rigidbodies;
	std::vector<QuadNode*> quadTreeLeafNodes;
	for (Rigidbody* rb = (Rigidbody*)m_allocator.m_pool.start; rb != nullptr; rb = m_allocator.GetNextBody(rb)) {
		rigidbodies.push_back(rb);
		rb->m_acceleration = Vector2(0, -m_gravity / rb->m_mass);
		if (!(rb->m_isKinematic || rb->m_isSleeping)) rb->m_velocity += rb->m_acceleration * dt;
		rb->m_prevPos = rb->m_position;
		rb->m_position += rb->m_velocity * dt;
		rb->m_rotation += rb->m_angularVelocity * dt;
	}

	//Q-tree: Assume space time coherence. Space: Objects cannot have a velocity bigger than the extent of a Q-node. Time: If we know what Q-node we were on
	//previous frame, we know to only check against Q-nodes adjacent to it, up to a max of 9.
	//When to subdivide Q-node? When number of body checks in one bin would surpass number of body checks in multiple bins (assuming uniform division?)
	//+ checking each body against necessary bins (9 approx?)
	//#TODO: Qtree step
	m_quadTreeRoot.GetLeafNodes(quadTreeLeafNodes);
	for (int i = 0; i < quadTreeLeafNodes.size(); i++)
	{
		QuadNode* leafNode = quadTreeLeafNodes[i];
		for (int j = 0; j < rigidbodies.size(); j++)
		{
			//Figure which bin rigidbody is on
			Rigidbody* rb = rigidbodies[j];
			if (rb->IntersectWith(leafNode->m_topRight, leafNode->m_bottomLeft))
			{
				leafNode->m_ownedBodies.push_back(rb);
			}
		}
	}
	
	//#TODO You might test twice for bodies that are both part of two QuadNodes at the same time, this might be why m_ignoreSeparatingBodies should be true
	for (int i = 0; i < quadTreeLeafNodes.size(); i++)
	{
		QuadNode* leafNode = quadTreeLeafNodes[i];
		for (int j = 0; j < leafNode->m_ownedBodies.size(); j++)
		{
			Rigidbody* rb1 = leafNode->m_ownedBodies[j];
			for (int k = j + 1; k < leafNode->m_ownedBodies.size(); k++) 
			{
				Rigidbody* rb2 = leafNode->m_ownedBodies[k];
				Manifold currentManifold;
				//If both objects are sleeping/kinematic, skip test
				if ((rb1->m_isSleeping || rb1->m_isKinematic) && (rb2->m_isSleeping || rb2->m_isKinematic)) continue;
				if (rb1->IntersectWith(rb2, currentManifold))
				{
					//They collide during the frame, store
					m_currentManifolds.push_back(currentManifold);//add manifolds
				}
			}
		}
	}

	//Remember to clear qnodes m_ownedBodies for next frame
	for (int i = 0; i < quadTreeLeafNodes.size(); i++)
	{
		quadTreeLeafNodes[i]->m_ownedBodies.clear();
	}

	//Collision response, may displace objects directly for static collision resolution
	for (Manifold manifold : m_currentManifolds) ComputeResponse(manifold);
	//Sleep check
	for (int i = 0; i < rigidbodies.size(); i++) 
	{
		Rigidbody* rb = rigidbodies[i];
		if (!rb->m_isKinematic) {
			if ((rb->m_position - rb->m_prevPos).LengthSqr() < FLT_EPSILON) {
				rb->m_timeInSleep += dt;
				//If its static for two timesteps or more, put to sleep
				if (rb->m_timeInSleep >= m_timestep * 2) {
					rb->m_isSleeping = true;
					rb->m_velocity = Vector2();
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
	avgContactPoint /= (decimal)manifold.numContactPoints;

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
		decimal dispFactor = (rb1->m_isKinematic) ? 0 : (rb2->m_isKinematic) ? 1 : 0.5f;
		rb1->m_position += pen * n * dispFactor;
		rb2->m_position -= pen * n * (1 - dispFactor);
	}
	if (m_ignoreSeparatingBodies) {
		if (vbaDotN > 0) return;//Possibly log this, helps solve interpenetration after response, by ignoring separating bodies
	}
	assert(vbaDotN < 0);
	decimal num = -(1 + e) * vbaDotN;
	decimal denom = invMassA + invMassB + Pow(raP.Dot(n), 2) * invIA + Pow(rbP.Dot(n), 2) * invIB;
	decimal impulse = num / denom;

	assert(impulse > 0);
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

//Go through custom allocator
Circle* Solver::CreateCircle(decimal rad, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	// Create the collision body, presumably a pool has been created beforehand
	Circle* circle = new (m_allocator.AllocateBody(sizeof(Circle))) Circle(rad, pos, rot, vel, angVel, mass, e, isKinematic);
	return circle;
}

Capsule* Solver::CreateCapsule(decimal length, decimal rad, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	Capsule* capsule = new (m_allocator.AllocateBody(sizeof(Capsule))) Capsule(length, rad, pos, rot, vel, angVel, mass, e, isKinematic);
	return capsule;
}

OrientedBox* Solver::CreateOrientedBox(math::Vector2 halfExtents, math::Vector2 pos, decimal rot, math::Vector2 vel, decimal angVel, decimal mass, decimal e, bool isKinematic)
{
	OrientedBox* obb = new (m_allocator.AllocateBody(sizeof(OrientedBox))) OrientedBox(halfExtents, pos, rot, vel, angVel, mass, e, isKinematic);
	return obb;
}

