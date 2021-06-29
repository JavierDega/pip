#pragma once

#include <vector>

#include "pipmath.h"
#include "Rigidbody.h"
#include "DefaultAllocator.h"
#include "QuadNode.h"

class Solver
{
public:
	Solver();
	~Solver();
	void Update(decimal dt);//Updates the time and executes fixed timestep Step();
	void ContinuousStep(decimal dt);//#Not supported: CCD Step physics forward
	void Step(decimal dt);// Discrete step
	void ComputeResponse(const pipmath::Manifold& manifold);
	Handle CreateCircle(decimal rad = 1.0f, pipmath::Vector2 pos = pipmath::Vector2(), decimal rot = 0.0f,
	 pipmath::Vector2 vel = pipmath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	Handle CreateCapsule(decimal length = 1.0f, decimal rad = 1.0f, pipmath::Vector2 pos = pipmath::Vector2(), decimal rot = 0.0f,
	 pipmath::Vector2 vel = pipmath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	Handle CreateOrientedBox(pipmath::Vector2 halfExtents = pipmath::Vector2(1.f, 1.f), pipmath::Vector2 pos = pipmath::Vector2(),
	 decimal rot = 0.0f, pipmath::Vector2 vel = pipmath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f,
	 bool isKinematic = false);
public:
	DefaultAllocator m_allocator;
	QuadNode m_quadTreeRoot;
	bool m_continuousCollision, m_stepMode, m_stepOnce, m_quadTreeSubdivision, m_ignoreSeparatingBodies, m_staticResolution,
	 m_logCollisionInfo, m_frictionModel;//#Bit field?
	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	std::vector<pipmath::Manifold> m_currentManifolds;
};
