#include <iostream>
#include <assert.h>
#include <float.h>
#include <algorithm>

#include "Solver.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

using namespace std;
using namespace PipMath;

Solver::Solver(BaseAllocator* allocator)
	: m_stepMode(false), m_stepOnce(false), m_quadTreeSubdivision(false), m_logCollisionInfo(false), m_frictionModel(true), m_allocator(allocator),
	m_quadTreeRoot(Vector2(10, 10), Vector2(-10, -10)), m_accumulator(0.f), m_timestep(0.02f), m_gravity(9.8f), m_airViscosity(0.133f)
{
}

Solver::~Solver()
{
	delete m_allocator;
	m_allocator = nullptr;
}

void Solver::Update(decimal dt)
{
	//Step through mem allocated bodies

	//Fixed timestep with accumulator (50fps)
	if (m_stepMode) {
		if (m_stepOnce) {
			Step(m_timestep);
			m_stepOnce = false;
		}
	}
	else {
		m_accumulator += dt;
		if (m_accumulator > 0.2f) m_accumulator = 0.2f;
		while (m_accumulator > m_timestep) {
			Step(m_timestep);
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
	//#Move code onto pool iterator
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
	//Integration
	std::vector<Rigidbody*> rigidbodies;
	assert(m_allocator);
	for (Rigidbody* rb = m_allocator->GetFirstBody(); rb != nullptr; rb = m_allocator->GetNextBody(rb)) {
		rigidbodies.push_back(rb);
		rb->m_acceleration += Vector2(0, -m_gravity / rb->m_mass);
		rb->m_acceleration -= m_airViscosity * rb->m_velocity / rb->m_mass;
		rb->m_angularAccel -= m_airViscosity * rb->m_angularVelocity / rb->m_mass; 
		if (!(rb->m_isKinematic || rb->m_isSleeping)) {
			rb->m_velocity += rb->m_acceleration * dt;
			rb->m_angularVelocity += rb->m_angularAccel * dt;	
		}
		rb->m_prevPos = rb->m_position;
		rb->m_prevRot = rb->m_rotation;
		rb->m_position += rb->m_velocity * dt;
		rb->m_rotation += rb->m_angularVelocity * dt;
		rb->m_acceleration = Vector2();
		rb->m_angularAccel = 0;
	}

	//Q-tree: Assume space time coherence. Space: Objects cannot have a velocity bigger than the extent of a Q-node. Time: If we know what Q-node we were on
	//previous frame, we know to only check against Q-nodes adjacent to it, up to a max of 9.
	//When to subdivide Q-node? When number of body checks in one bin would surpass number of body checks in multiple bins (assuming uniform division?)
	//+ checking each body against necessary bins (9 approx?)
	std::vector<QuadNode*> quadTreeLeafNodes;
	m_quadTreeRoot.GetLeafNodes(quadTreeLeafNodes);

	for (int i = 0; i < quadTreeLeafNodes.size(); i++)
	{
		QuadNode* leafNode = quadTreeLeafNodes[i];
		if (!leafNode->m_ownedBodies.empty()) leafNode->m_ownedBodies.clear();
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

	m_currentManifolds.clear();
	//You might test twice for bodies that are both part of two QuadNodes at the same time, which is why m_ignoreSeparatingBodies should be true
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

	//Collision response, may displace objects directly for static collision resolution
	for (const Manifold& manifold : m_currentManifolds) ComputeResponse(manifold);
	//Sleep check
	for (int i = 0; i < rigidbodies.size(); i++)
	{
		Rigidbody* rb = rigidbodies[i];
		if (!rb->m_isKinematic)
		{
			if ((rb->m_position - rb->m_prevPos).LengthSqr() <= 0.001 * 0.001 &&
			(rb->m_rotation - rb->m_prevRot) <= 0.001)
			{
				//#Issues with bodies going to sleep when they shouldnt on fixed point mode
				rb->m_timeInSleep += dt;
				//If its static for two timesteps or more, put to sleep
				if (!rb->m_isSleeping && rb->m_timeInSleep >= m_timestep * 2)
				{
					rb->m_isSleeping = true;
					rb->m_velocity = Vector2();
					rb->m_angularVelocity = 0;
				}
			}
			else
			{
				if (rb->m_isSleeping)
				{
					rb->m_isSleeping = false;
				}
				if (rb->m_timeInSleep > 0)
				{
					rb->m_timeInSleep = 0;
				}
			}
		}
	}

	//Before clearing their ownedBodies we wanna know which qnodes need merging/subdividing
	if (m_quadTreeSubdivision)
	{
		std::vector<QuadNode*> quadTreeLeafParentNodes;
		for (int i = 0; i < quadTreeLeafNodes.size(); i++)
		{
			QuadNode* leafNode = quadTreeLeafNodes[i];
			QuadNode* leafNodeParent = leafNode->m_owner;
			if (leafNodeParent)//If its the root node it will have no parent
			{
				if (std::find(quadTreeLeafParentNodes.begin(), quadTreeLeafParentNodes.end(), leafNodeParent) == quadTreeLeafParentNodes.end())
				{
					//Its not contained already in the vector, add it
					quadTreeLeafParentNodes.push_back(leafNodeParent);
				}
			}
		}
		for (int i = 0; i < quadTreeLeafNodes.size(); i++)
		{
			QuadNode* leafNode = quadTreeLeafNodes[i];
			leafNode->TrySubdivide();
		}

		for (int i = 0; i < quadTreeLeafParentNodes.size(); i++)
		{
			QuadNode* leafParent = quadTreeLeafParentNodes[i];
			leafParent->TryMerge();
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
	decimal invMassA = rb1->m_isKinematic ? 0 : (decimal)1 / rb1->m_mass;
	decimal invMassB = rb2->m_isKinematic ? 0 : (decimal)1 / rb2->m_mass;
	decimal invIA = rb1->m_isKinematic ? 0 : (decimal)1 / rb1->m_inertia;//Inverse Inertia
	decimal invIB = rb2->m_isKinematic ? 0 : (decimal)1 / rb2->m_inertia;
	//Make multiple contact points work by caching velocities and then summing local impulses
	Vector2 resultVelA = rb1->m_velocity;
	Vector2 resultVelB = rb2->m_velocity;
	decimal resultAngVelA = rb1->m_angularVelocity;
	decimal resultAngVelB = rb2->m_angularVelocity;
	decimal e = Sqrt(rb1->m_e * rb2->m_e); //Coefficient of restitution
	Vector2 avgContactPoint = Vector2();
	//#Not final may be a better way of dealing with multiple contact points
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
	Vector2 vba = va - vb;
	decimal vbaDotN = vba.Dot(n);
	//Generic solution that uses manifold's penetration to displace rigidbodies along the normal
	//If we do this, will kinematic objects get displaced by much?
	decimal dispFactor = (rb1->m_isKinematic) ? 0 : (rb2->m_isKinematic) ? 1 : 0.5f;
	rb1->m_position += pen * n * dispFactor;
	rb2->m_position -= pen * n * ((decimal)1 - dispFactor);
	//assert(vbaDotN < 0)
	if (vbaDotN >= 0) return;//Possibly log this, helps solve interpenetration after response, by ignoring separating bodies
	decimal num = -((decimal)1 + e) * vbaDotN;
	decimal denom = invMassA + invMassB + Pow(raP.Dot(n), 2) * invIA + Pow(rbP.Dot(n), 2) * invIB;
	decimal impulseReactionary = num / denom;
	assert(impulseReactionary > 0);
	resultVelA += impulseReactionary * n * invMassA;
	resultAngVelA += raP.Dot(impulseReactionary * n) * invIA;
	resultVelB -= impulseReactionary * n * invMassB;
	resultAngVelB -= rbP.Dot(impulseReactionary * n) * invIB;

	//Static and kinetic friction model: Generate a force at the contact point of magnitude m_frictionCoefficient = 0.03f or double if object is sleeping
	//If friction is bigger than this force, scale friction back to match
	//https://en.wikipedia.org/wiki/Collision_response#:~:text=Impulse%2Dbased%20friction%20model,-Coulomb%20friction%20model&text=The%20Coulomb%20friction%20model%20effectively,the%20static%20configuration%20is%20maintained.
	//Coulomb impulse based friction model
	if (m_frictionModel)
	{
		/// <summary>
		///#3D formula
		///jr = magnitude of impulse acting along normal n
		///t = tangent vector, orthogonal to n
		///jf = frictional impulse, which can be static js or kinetic jk
		///us = coefficient of static friction
		///uk = coefficient of kinetic friction
		///js = us*jr
		///jk = uk*jr
		///jf = -jk*t o -m(vba.Dot(t))*t  
		/// <param name="manifold"></param>
		/// <summary>
		/// #2D:
		/// jr = impulse
		/// jf = frictionalImpulse, static (js) or kinetic (jk)
		/// us = coefficient of static friction
		/// uk = coefficient of kinetic friction
		///  
		/// </summary>
		/// <param name="manifold"></param>

		//va = resultVelA + resultAngVelA * raP;
		//vb = resultVelB + resultAngVelB * rbP;
		//vba = va - vb;
		//vbaDotN = vba.Dot(n);
		decimal kFrictionCoefficient = Sqrt(rb1->m_kFriction * rb2->m_kFriction);
		decimal sFrictionCoefficient = kFrictionCoefficient * 2;
		Vector2 t = (vba - n * vbaDotN);//Tangential component of relative (linear) velocities
		Vector2 tangentDir = t.EqualsEps(Vector2(0, 0), PIP_TESTS_EPSILON) ? Vector2(0, 0) : t.Normalized();
		//Figure out what part of impulseReactionary was applied through t
		decimal impulseFrictional1 = impulseReactionary * (rb1->m_isSleeping ? sFrictionCoefficient : kFrictionCoefficient);
		decimal impulseFrictional2 = impulseReactionary * (rb2->m_isSleeping ? sFrictionCoefficient : kFrictionCoefficient);
		resultVelA -= impulseFrictional1 * tangentDir * invMassA;
		resultAngVelA -= raP.Dot(impulseFrictional1 * tangentDir) * invIA;
		resultVelB += impulseFrictional2 * tangentDir * invMassB;
		resultAngVelB += rbP.Dot(impulseFrictional2 * tangentDir) * invIB;
	}

	if (m_logCollisionInfo) {
		cout << "-----------------------------------PiP Log - Collision Response Info-------------------------------" << endl
			<< "Normal: " << n << endl
			<< "ra (rb1 to contact point): " << ra << endl
			<< "rb (rb2 to contact point): " << rb << endl
			<< "rb1 velocity: " << rb1->m_velocity << " velocity at contact point: " << va << endl
			<< "rb2 velocity: " << rb2->m_velocity << " velocity at contact point: " << vb << endl
			<< "relative velocity vba: " << vba << "combined coefficient of restituion (e): " << e << endl
			<< "impulseReactionary = " << num << " / " << denom << " = " << impulseReactionary << endl
			<< "Result: velA: x(" << resultVelA << " velB = " << resultVelB << endl
			<< "angVelA = " << resultAngVelA << " angVelB = " << resultAngVelB << endl;
	}

	rb1->m_velocity = resultVelA;
	rb1->m_angularVelocity = resultAngVelA;
	rb2->m_velocity = resultVelB;
	rb2->m_angularVelocity = resultAngVelB;
}

//Go through custom allocator
int Solver::CreateCircle(Handle& handle, decimal rad, PipMath::Vector2 pos, decimal rot, PipMath::Vector2 vel, decimal angVel, decimal mass,
	decimal e, bool isKinematic, decimal kFriction)
{
	Handle circleHandle;
	assert(m_allocator);
	Circle* circle = new (m_allocator->AllocateBody(sizeof(Circle), circleHandle)) Circle(rad, pos, rot, vel, angVel, mass, e, isKinematic, kFriction);
	return circle ? 0 : -1;
}

int Solver::CreateCapsule(Handle& handle, decimal length, decimal rad, PipMath::Vector2 pos, decimal rot, PipMath::Vector2 vel, decimal angVel, decimal mass,
	decimal e, bool isKinematic, decimal kFriction)
{
	Handle capsuleHandle;
	assert(m_allocator);
	Capsule* capsule = new (m_allocator->AllocateBody(sizeof(Capsule), capsuleHandle)) Capsule(length, rad, pos, rot, vel, angVel, mass, e, isKinematic,
	kFriction);
	return capsule ? 0 : -1;
}

int Solver::CreateOrientedBox(Handle& handle, PipMath::Vector2 halfExtents, PipMath::Vector2 pos, decimal rot, PipMath::Vector2 vel, decimal angVel,
	decimal mass, decimal e, bool isKinematic, decimal kFriction)
{
	Handle obbHandle;
	assert(m_allocator);
	OrientedBox* obb = new (m_allocator->AllocateBody(sizeof(OrientedBox), obbHandle)) OrientedBox(halfExtents, pos, rot, vel, angVel, mass, e, isKinematic,
	kFriction);
	return obb ? 0 : -1;
}

