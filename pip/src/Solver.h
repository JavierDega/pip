#pragma once

#include <vector>

#include "PipMath.h"
#include "Rigidbody.h"
#include "OrientedBox.h"
#include "DefaultAllocator.h"
#include "BaseAllocator.h"
#include "QuadNode.h"

class Solver
{
public:
	Solver(BaseAllocator* allocator = new DefaultAllocator(50*sizeof(OrientedBox)));
	~Solver();
	void Update(decimal dt);//Updates the time and executes physics step
	int CreateCircle(Handle& handle, decimal rad = 1.0f, PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.0f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false,
	 decimal kFriction = 0.03f);
	int CreateCapsule(Handle& handle, decimal length = 1.0f, decimal rad = 1.0f, PipMath::Vector2 pos = PipMath::Vector2(), decimal rot = 0.0f,
	 PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false,
	 decimal kFriction = 0.03f);
	int CreateOrientedBox(Handle& handle, PipMath::Vector2 halfExtents = PipMath::Vector2(1.f, 1.f), PipMath::Vector2 pos = PipMath::Vector2(),
	 decimal rot = 0.0f, PipMath::Vector2 vel = PipMath::Vector2(), decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false,
	 decimal kFriction = 0.03f);
	void ComputeResponse(const PipMath::Manifold& manifold);
private:
	void ContinuousStep(decimal dt);//#Not supported: Continuous collision physics step
	void Step(decimal dt);// Discrete step
public:
	BaseAllocator* m_allocator;
	QuadNode m_quadTreeRoot;
	bool m_stepMode, m_stepOnce, m_logCollisionInfo, m_frictionModel;//#Bit field?
	decimal m_timestep, m_airViscosity;
	PipMath::Vector2 m_gravity;
	std::vector<PipMath::Manifold> m_currentManifolds;
private:
	decimal m_accumulator;
};
