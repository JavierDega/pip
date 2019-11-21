#include "Solver.h"
#if USE_FIXEDPOINT
using namespace fp64;
#endif

using namespace math;

Solver::Solver()
	: m_continuousCollision(true), m_accumulator(0.f), m_timestep(0.02f), m_gravity(m_timestep * 9.8f)
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
	m_accumulator += dt;
	if (m_accumulator > 0.2f) m_accumulator = 0.2f;
	while (m_accumulator > m_timestep) {
		(m_continuousCollision) ? ContinuousStep(m_timestep) : Step(m_timestep);
		m_accumulator -= m_timestep;
	}
	//@UP TO THE GRAPHICS APPLICATION:
	//To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	//float alpha = m_accumulator / m_timestep;
}

void Solver::ContinuousStep(decimal timestep) 
{
	//We need to have up to date velocities to perform sweeps
	//1: Perform sweeps, storing collision deltas
	//2: Only acknowledge first collision time
	//3: Step everything upto first collision time
	//4: Compute responses for first colliding pair, everything else retains velocity
	//5: Reperform sweeps  (for(i){for (j = i + 1; ..)}
	//6: If new collisions, repeat
	//7: Apply gravity forces (Collisions are impulse based, friction is force based) *optionally do this at step 0
	decimal dt = (decimal)timestep;
	while ( dt > (decimal) 0.f) {
		decimal firstCollision = 1;//Normalized dt
		std::pair<Rigidbody*, Rigidbody*> firstCollidingPair;
		//Upwards collision check
		for (int i = 0; i < m_rigidbodies.size(); i++) {
			for (int j = i + 1; j < m_rigidbodies.size(); j++) {
				if (decimal t = m_rigidbodies[i]->ComputeSweep(m_rigidbodies[j], dt)) {
					//They collide during the frame, store
					if ( t > 0 && t <= firstCollision) {
						firstCollision = t;
						firstCollidingPair.first = m_rigidbodies[i];
						firstCollidingPair.second = m_rigidbodies[j];
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
		if (firstCollidingPair.first) {
			ComputeResponse(firstCollidingPair.first, firstCollidingPair.second);
		}
	}
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		//Semi euler integration
		Rigidbody* rb = m_rigidbodies[i];
		rb->m_acceleration = Vector2( 0, -m_gravity / rb->m_mass);
		rb->m_velocity += rb->m_acceleration;
	}
}

void Solver::Step(decimal dt)
{

	//Upwards collision check
	std::vector<std::pair<Rigidbody*, Rigidbody*>> collidingPairs;
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		for (int j = i + 1; j < m_rigidbodies.size(); j++) {
			if (!m_rigidbodies[i]->ComputeIntersect(m_rigidbodies[j], dt)) {
				//They collide during the frame, store
				collidingPairs.push_back(std::make_pair(m_rigidbodies[i], m_rigidbodies[j]));//add manifolds
			}
		}
	}

	for (std::pair<Rigidbody*, Rigidbody*> collidingPair : collidingPairs) {
		ComputeResponse(collidingPair.first, collidingPair.second);
	}

	//Integration
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		//Semi euler integration
		Rigidbody* rb = m_rigidbodies[i];
		rb->m_acceleration = Vector2(0, -m_gravity / rb->m_mass);
		rb->m_velocity += rb->m_acceleration;
		rb->m_position += rb->m_velocity;
		//TODO: Rotations
	}
}

void Solver::ComputeResponse(Rigidbody * rb1, Rigidbody * rb2)
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
circle2.setMovementVector(v2');*/

	//Center of rb1 to center of rb2
	Vector2 n = rb2->m_position - rb1->m_position;
	n.Normalize();

	// Find the length of the component of each of the movement vectors along n.
	decimal a1 = Vector2::Dot(rb1->m_velocity, n);
	decimal a2 = Vector2::Dot(rb2->m_velocity, n);

	// Using the optimized version,
	// optimizedP =  2(a1 - a2)
	//              -----------
	//                m1 + m2
	decimal optimizedP = ((decimal)2.f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

	// Calculate v1', the new movement vector of circle1
	// v1' = v1 - optimizedP * m2 * n
	rb1->m_velocity -= n * optimizedP * rb2->m_mass;
	// Calculate v1', the new movement vector of circle1
	// v2' = v2 + optimizedP * m1 * n
	rb2->m_velocity += n * optimizedP * rb1->m_mass;
}

Rigidbody * Solver::AddBody(Rigidbody * rb)
{
	//Create default sphere and pass reference, using allocator
	m_rigidbodies.push_back(rb);
	return m_rigidbodies.back();
}
