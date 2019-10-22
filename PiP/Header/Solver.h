#pragma once
#include "Rigidbody.h"
#include "StackAllocator.h"

#include <vector>
#include <map>

class Solver
{
public:
	Solver();
	~Solver();

	void Update(float dt);//Updates the time and executes fixed timestep Step();
	void Step(float dt);//Steps physics forward
	float ComputeSweep(Rigidbody * rb1, Rigidbody * rb2, float dt);
	void ComputeResponse(Rigidbody * rb1, Rigidbody * rb2);
	Rigidbody * AddBody();

	float m_accumulator;
	float m_timestep = 0.02f; 
	std::vector<Rigidbody*> m_rigidbodies;

	std::map<std::pair<Rigidbody*, Rigidbody*>, float> m_collidingpairs;
};
