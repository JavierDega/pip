#pragma once
#include "PiPMath.h"
#include "Circle.h"
#include "StackAllocator.h"

#include <vector>

class Solver
{
public:
	Solver();
	~Solver();

	void Update(decimal dt);//Updates the time and executes fixed timestep Step();
	void ContinuousStep(decimal dt);//CCD Step physics forward
	void Step(decimal dt);// Discrete step
	void ComputeResponse(Rigidbody * rb1, Rigidbody * rb2);
	Rigidbody * AddBody(Rigidbody * rb);

	bool m_continuousCollision;
	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	std::vector<Rigidbody*> m_rigidbodies;
};
