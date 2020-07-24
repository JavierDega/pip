#include "QuadNode.h"

using namespace math;

QuadNode::QuadNode(Vector2 topRight, Vector2 bottomLeft, bool isLeaf)
	: m_topRight(topRight), m_bottomLeft(bottomLeft), m_isLeaf(isLeaf)
{
	m_children = nullptr;
}

QuadNode::~QuadNode()
{
}

void QuadNode::Subdivide()
{
	m_isLeaf = false;
	m_children = new QuadNode[4];
	Vector2 midPoint = m_topRight + (m_bottomLeft - m_topRight)/2;
	//Nodes: top left
	m_children[0].m_topRight = Vector2(midPoint.x, m_topRight.y);
	m_children[0].m_bottomLeft = Vector2(m_bottomLeft.x, midPoint.y);
	//top right
	m_children[1].m_topRight = Vector2(m_topRight.x, m_topRight.y);
	m_children[1].m_bottomLeft = Vector2(midPoint.x, midPoint.y);
	//bottom left
	m_children[2].m_topRight = Vector2(midPoint.x, midPoint.y);
	m_children[2].m_bottomLeft = Vector2(m_bottomLeft.x, m_bottomLeft.y);
	//bottom right
	m_children[3].m_topRight = Vector2(m_topRight.x, midPoint.y);
	m_children[3].m_bottomLeft = Vector2(midPoint.x, m_bottomLeft.y);
}

unsigned int QuadNode::GetLeafNodes(std::vector<QuadNode*>& leafNodes)
{
	unsigned int leafCount = 0;
	if (m_isLeaf) {
		leafNodes.push_back(this);
		leafCount = 1;
	}
	else {
		for (int i = 0; i < 4; i++) {
			QuadNode currentQNode = m_children[i];
			leafCount += currentQNode.GetLeafNodes(leafNodes);
		}
	}
	return leafCount;
}

