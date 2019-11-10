#pragma once
#include "PiPMath.h"
#include "Rigidbody.h"
#include "StackAllocator.h"

#include <vector>

class Solver
{
public:
	Solver();
	~Solver();

	void Update(decimal dt);//Updates the time and executes fixed timestep Step();
	void Step(decimal dt);//Steps physics forward
	decimal ComputeSweep(Rigidbody * rb1, Rigidbody * rb2, decimal dt);
	void ComputeResponse(Rigidbody * rb1, Rigidbody * rb2);
	Rigidbody * AddBody(math::Vector2 pos = math::Vector2(), decimal rot = (decimal)0.f, math::Vector2 vel = math::Vector2(), 
		decimal angVel = (decimal)0.f, math::Vector2 accel = math::Vector2(), decimal rad = (decimal)1.f,
		decimal mass = (decimal)1.f);

	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	std::vector<Rigidbody*> m_rigidbodies;
};
