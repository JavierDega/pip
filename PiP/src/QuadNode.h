#pragma once
#include "PiPMath.h"
#include <vector>

class QuadNode
{
public:
	QuadNode(math::Vector2 topRight = math::Vector2(), math::Vector2 bottomLeft = math::Vector2(), bool isLeaf = true);
	~QuadNode();

	void Subdivide();//Adds 4 children in four containers inside parent
	void Merge();//Deletes children, becomes a leaf node (Shouldn't be called on QNodes too far up in the hierarchy? Should only merge one level per frame (Spacetime coherency should help this))
	unsigned int GetLeafNodes(std::vector<QuadNode*>& leafNodes);
	void Update();//Only runs if this is a leaf node, then it checks upwards to see if it should shrink children nodes

	math::Vector2 m_topRight;
	math::Vector2 m_bottomLeft;
	bool m_isLeaf;
	std::vector<Rigidbody*> m_ownedBodies;
	QuadNode* m_children;
	QuadNode* m_owner;
};

