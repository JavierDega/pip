#pragma once

#include <vector>

#include "PipMath.h"
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
	void ComputeResponse(const PipMath::Manifold& manifold);
	int CreateCircle(Handle& handle, decimal rad = 1.0f, PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.0f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	int CreateCapsule(Handle& handle, decimal length = 1.0f, decimal rad = 1.0f, PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.0f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	int CreateOrientedBox(Handle& handle, PipMath::Vector2 halfExtents = PipMath::Vector2(1.f, 1.f), PipMath::Vector2 pos = PipMath::Vector2(),
	 decimal rot = 0.0f, PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f,
	 bool isKinematic = false);
public:
	DefaultAllocator m_allocator;
	QuadNode m_quadTreeRoot;
	bool m_continuousCollision, m_stepMode, m_stepOnce, m_quadTreeSubdivision, m_staticResolution, m_logCollisionInfo, 
	m_frictionModel;//#Bit field?
	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	decimal m_airViscosity;
	std::vector<PipMath::Manifold> m_currentManifolds;
};
