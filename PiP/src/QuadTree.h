#pragma once
#include "PiPMath.h"

class QuadTree
{
public:
	QuadTree(math::Vector2 topRight = math::Vector2(), math::Vector2 bottomLeft = math::Vector2(), bool isLeaf = true);
	~QuadTree();

	void Subdivide();//Adds 4 children in four containers inside parent

	math::Vector2 m_topRight;
	math::Vector2 m_bottomLeft;
	bool m_isLeaf;
	QuadTree* m_children;
};

