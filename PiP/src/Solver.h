#pragma once
#include <vector>

#include "PiPMath.h"
#include "Circle.h"
#include "Capsule.h"
#include "StackAllocator.h"

class Solver
{
public:
	Solver();
	~Solver();

	void Update(decimal dt);//Updates the time and executes fixed timestep Step();
	void ContinuousStep(decimal dt);//CCD Step physics forward
	void Step(decimal dt);// Discrete step
	void ComputeResponse(const math::Manifold& manifold);
	Rigidbody * AddBody(Rigidbody * rb);

	bool m_continuousCollision;
	bool m_stepMode;
	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	std::vector<Rigidbody*> m_rigidbodies;
};
