#pragma once

#include <vector>

#include "PipMath.h"

class QuadNode
{
public:
	QuadNode(PipMath::Vector2 topRight = PipMath::Vector2(), PipMath::Vector2 bottomLeft = PipMath::Vector2(),
	 bool isLeaf = true);
	~QuadNode();
	unsigned int GetLeafNodes(std::vector<QuadNode*>& leafNodes);//RECURSIVE
	void TrySubdivide();//See if conditions are fulfilled for subdividing this leaf node into 4 children
	void TryMerge();//See if conditions are fulfilled for merging children nodes on this leaf nodes parent
	QuadNode* GetOwner();
	PipMath::Vector2 GetTopRight();
	PipMath::Vector2 GetBottomLeft();
public:
	std::vector<Rigidbody*> m_ownedBodies;//Data owned by memory allocator
private:
	PipMath::Vector2 m_topRight;
	PipMath::Vector2 m_bottomLeft;	
	QuadNode* m_owner;
	bool m_isLeaf;
	QuadNode* m_children;
};

