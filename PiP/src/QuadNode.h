#pragma once
#include "PiPMath.h"
#include <vector>

class QuadNode
{
public:
	QuadNode(math::Vector2 topRight = math::Vector2(), math::Vector2 bottomLeft = math::Vector2(), bool isLeaf = true);
	~QuadNode();

	void Subdivide();//Adds 4 children in four containers inside parent
	unsigned int GetLeafNodes(std::vector<QuadNode*>& leafNodes);

	math::Vector2 m_topRight;
	math::Vector2 m_bottomLeft;
	bool m_isLeaf;
	std::vector<Rigidbody*> m_ownedBodies;
	QuadNode* m_children;
};

