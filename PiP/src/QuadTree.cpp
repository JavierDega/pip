#include "QuadTree.h"

using namespace math;

QuadTree::QuadTree(Vector2 topRight, Vector2 bottomLeft, bool isLeaf)
	: m_topRight(topRight), m_bottomLeft(bottomLeft), m_isLeaf(isLeaf)
{
	m_children = nullptr;
}

QuadTree::~QuadTree()
{
}

void QuadTree::Subdivide()
{
	m_isLeaf = false;
	m_children = new QuadTree[4];
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
