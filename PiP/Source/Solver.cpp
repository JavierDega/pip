#include "stdafx.h"
#include "Header\Solver.h"

using namespace fp64;

Solver::Solver()
{
	m_accumulator = 0.f;
	m_timestep = 0.02f;
}


Solver::~Solver()
{
	m_rigidbodies.clear();
}

void Solver::Update(float dt)
{
	//Step through mem allocated bodies

	//Fixed timestep with accumulator (50fps)
	m_accumulator += dt;
	if (m_accumulator > 0.2f) m_accumulator = 0.2f;
	while (m_accumulator > m_timestep) {
		Step(m_timestep);
		m_accumulator -= m_timestep;
	}
	//@UP TO THE GRAPHICS APPLICATION:
	//To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	//float alpha = m_accumulator / m_timestep;


}

void Solver::Step(float timestep) 
{
	//We need to have up to date velocities to perform sweeps
	float firstCollision = 1;//Normalized dt
	std::pair<Rigidbody*, Rigidbody*> firstCollidingPair;
	//Upwards collision check
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		for (int j = i + 1; j < m_rigidbodies.size(); j++) {
			if (float t = ComputeSweep(m_rigidbodies[i], m_rigidbodies[j], timestep)) {
				//They collide during the frame, store
				if (t < firstCollision) {
					firstCollision = t;
					firstCollidingPair.first = m_rigidbodies[i];
					firstCollidingPair.second = m_rigidbodies[j];
				}
			}
		}
	}

	//If there was collision
	if (firstCollision != 1) {
		//Step everything upto collision time
		float timeToStep = timestep * firstCollision;
		for (int i = 0; i < m_rigidbodies.size(); i++) {
			//Step according to their velocities (Don't apply gravity or frictional accelerations).
			Rigidbody * rb = m_rigidbodies[i];
			rb->m_position += rb->m_velocity*timeToStep;
			rb->m_rotation += rb->m_angularVelocity*timeToStep;
		}
		ComputeResponse(firstCollidingPair.first, firstCollidingPair.second);
	}
	//1: Perform sweeps, storing collision deltas
	//2: Only acknowledge first collision time
	//3: Step everything upto first collision time
	//4: Compute responses for first colliding pair, everything else retains velocity
	//5: Reperform sweeps  (for(i){for (j = i + 1; ..)}
	//6: If new collisions, repeat
	//7: Apply gravity forces (Collisions are impulse based, friction is force based) *optionally do this at step 0


}

float Solver::ComputeSweep(Rigidbody * rb1, Rigidbody * rb2, float dt) 
{
	//https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=2

	//A = A0 + U*VA
	//B = B0 + U*V

	//B(U) - A(U) = ra + rb

	const decimal ra = rb1->m_radius;
	const decimal rb = rb2->m_radius;

	const Vector2 va = rb1->m_velocity*dt;
	const Vector2 vb = rb2->m_velocity*dt;

	const Vector2 ab = rb2->m_position - rb1->m_position;
	const Vector2 vab = vb - va;

	const decimal rab = ra + rb;

	const decimal a = Vector2::Dot(vab, vab);
	const decimal b = (decimal)2.f * Vector2::Dot(vab, ab);
	const decimal c = Vector2::Dot(ab, ab) - rab * rab;

	const decimal q = b * b - (decimal)4.f * a*c;
	if (q < (decimal)0.f) {
		return 0;//No root, no collision
	}
	else {
		const decimal sq = Fp64::Sqrt(q, 3);
		const decimal d = decimal(1.f) / ((decimal)2.f*a);
		const decimal root1 = ((decimal)-1.f*b + sq)*d;
		const decimal root2 = ((decimal)-1.f*b - sq)*d;
		if (root1 <= root2) return root1;

	}
}

void Solver::ComputeResponse(Rigidbody * rb1, Rigidbody * rb2)
{

}
Rigidbody * Solver::AddBody()
{
	//Create default sphere and pass reference, using allocator
	return nullptr;
}
