#pragma once
#include "Rigidbody.h"
#include "StackAllocator.h"

#include <vector>

class Solver
{
public:
	Solver();
	~Solver();

	void Update(float dt);//Updates the time and executes fixed timestep Step();
	void Step(float dt);//Steps physics forward
	decimal ComputeSweep(Rigidbody * rb1, Rigidbody * rb2, decimal dt);
	void ComputeResponse(Rigidbody * rb1, Rigidbody * rb2);
	Rigidbody * AddBody(Vector2 pos = Vector2(), decimal rot = (decimal)0.f, Vector2 vel = Vector2(), decimal angVel = (decimal)0.f, Vector2 accel = Vector2(), decimal rad = (decimal)1.f,
		decimal mass = (decimal)1.f);

	float m_accumulator;
	float m_timestep = 0.02f; 
	std::vector<Rigidbody*> m_rigidbodies;
};
