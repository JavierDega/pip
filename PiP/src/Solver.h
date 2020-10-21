#pragma once
#include <vector>

#include "PiPMath.h"
#include "Rigidbody.h"
#include "DefaultAllocator.h"
#include "QuadNode.h"

class Solver
{
public:
	Solver();
	~Solver();

	void Update(decimal dt);//Updates the time and executes fixed timestep Step();
	void ContinuousStep(decimal dt);//CCD Step physics forward
	void Step(decimal dt);// Discrete step
	void ComputeResponse(const math::Manifold& manifold);
	Circle* CreateCircle(decimal rad = 1.0f, math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	Capsule* CreateCapsule(decimal length = 1.0f, decimal rad = 1.0f, math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);
	OrientedBox* CreateOrientedBox(math::Vector2 halfExtents = math::Vector2(1.f, 1.f), math::Vector2 pos = math::Vector2(), decimal rot = 0.0f, math::Vector2 vel = math::Vector2(),
		decimal angVel = 0.0f, decimal mass = 1.0f, decimal e = 1.f, bool isKinematic = false);

	DefaultAllocator m_allocator;
	QuadNode m_quadTreeRoot;
	bool m_continuousCollision, m_stepMode, m_stepOnce, m_quadTreeSubdivision, m_ignoreSeparatingBodies, m_staticResolution, m_logCollisionInfo, m_frictionModel;
	decimal m_accumulator;
	decimal m_timestep;
	decimal m_gravity;
	std::vector<math::Manifold> m_currentManifolds;
};
