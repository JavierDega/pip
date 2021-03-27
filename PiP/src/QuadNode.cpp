#include "QuadNode.h"

#include <assert.h>

#define QNODE_MERGE_THRESHOLD 8 // 8 objects in 1 node = 28 tests. 8 objects in 4 nodes = 32 + 1*4 = 36 tests if fully balanced
#define QNODE_SUBDIVIDE_THRESHOLD 12 //12 objects in 1 node = 66 tests. 12 objects in 4 nodes = 48 + 3*4 = 60 tests if fully balanced

using namespace pipmath;


QuadNode::QuadNode(Vector2 topRight, Vector2 bottomLeft, bool isLeaf)
	: m_topRight(topRight), m_bottomLeft(bottomLeft), m_isLeaf(isLeaf), m_owner(nullptr), m_children (nullptr)
{
	bool debugBreak = false;
}

QuadNode::~QuadNode()
{
	delete[] m_children;
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
			QuadNode* currentQNode = m_children + i;
			leafCount += currentQNode->GetLeafNodes(leafNodes);
		}
	}
	return leafCount;
}

void QuadNode::TrySubdivide()
{
	assert(m_isLeaf && !m_children);//Assert were leaf node and thus have no children
	//Measure owned bodies
	if (m_ownedBodies.size() >= QNODE_SUBDIVIDE_THRESHOLD) 
	{
		if (!m_ownedBodies.empty()) m_ownedBodies.clear();
		m_isLeaf = false;
		m_children = new QuadNode[4];
		Vector2 midPoint = m_topRight + (m_bottomLeft - m_topRight) / 2;
		//Nodes: top left);
		m_children[0].m_owner = this;
		m_children[0].m_topRight = Vector2(midPoint.x, m_topRight.y);
		m_children[0].m_bottomLeft = Vector2(m_bottomLeft.x, midPoint.y);
		//top right
		m_children[1].m_owner = this;
		m_children[1].m_topRight = Vector2(m_topRight.x, m_topRight.y);
		m_children[1].m_bottomLeft = Vector2(midPoint.x, midPoint.y);
		//bottom left
		m_children[2].m_owner = this;
		m_children[2].m_topRight = Vector2(midPoint.x, midPoint.y);
		m_children[2].m_bottomLeft = Vector2(m_bottomLeft.x, m_bottomLeft.y);
		//bottom right
		m_children[3].m_owner = this;
		m_children[3].m_topRight = Vector2(m_topRight.x, midPoint.y);
		m_children[3].m_bottomLeft = Vector2(midPoint.x, m_bottomLeft.y);
	}
}

void QuadNode::TryMerge()
{
	//Assert were not a leaf node, return if our children just subdivided and thus are not leaf anymore, as they probably fulfill the merge threshold
	assert(!m_isLeaf);
	assert(m_children);
	
	if (!m_children[0].m_isLeaf) return;

	//Count children bodies see if they add up to threshold
	unsigned int childrenBodyTotal = 0;
	std::vector<QuadNode*> leafNodes;
	GetLeafNodes(leafNodes);
	for (int i = 0; i <leafNodes.size(); i++)
	{
		QuadNode* childLeaf = leafNodes[i];
		childrenBodyTotal += (unsigned int)childLeaf->m_ownedBodies.size();
	}
	if (childrenBodyTotal <= QNODE_MERGE_THRESHOLD)
	{
		delete[] m_children;//Should delete recursively
		m_children = nullptr;
		m_isLeaf = true;
	}
}


