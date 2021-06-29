#pragma once

#include <vector>

#include "pipmath.h"

class QuadNode
{
public:
	QuadNode(pipmath::Vector2 topRight = pipmath::Vector2(), pipmath::Vector2 bottomLeft = pipmath::Vector2(),
	 bool isLeaf = true);
	~QuadNode();
	unsigned int GetLeafNodes(std::vector<QuadNode*>& leafNodes);//RECURSIVE
	void TrySubdivide();//See if conditions are fulfilled for subdividing this leaf node into 4 children
	void TryMerge();//See if conditions are fulfilled for merging children nodes on this leaf nodes parent	
public:
	pipmath::Vector2 m_topRight;
	pipmath::Vector2 m_bottomLeft;
	bool m_isLeaf;
	std::vector<Rigidbody*> m_ownedBodies;//Data owned by memory allocator
	QuadNode* m_children;
	QuadNode* m_owner;
};

