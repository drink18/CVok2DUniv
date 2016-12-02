#include "cvBroadphaseBruteForce.h"

cvBroadphaseBruteForce::cvBroadphaseBruteForce(cvBroadphaseCInfo& cInfo)
	:cvBroadphase(cInfo)
{
}

void cvBroadphaseBruteForce::updateOneNode(cvBroadphaseHandle/* handle*/, const cvAabb&/* newAabb*/)
{

}

cvBroadphaseHandle cvBroadphaseBruteForce::addNode(const cvAabb& nodeAabb)
{
	cvBroadphaseHandle handle = m_Nodes.alloc();
	BPNode& node = m_Nodes.getAt(handle);
	node.m_aabb = nodeAabb;
	return handle;
}

void cvBroadphaseBruteForce::removeNode(cvBroadphaseHandle handle)
{
	m_Nodes.free(handle);
}

bool cvBroadphaseBruteForce::addPair(const cvBroadphaseHandle&/* handle1*/, const cvBroadphaseHandle&/* handle2*/)
{
	return false;
}


bool cvBroadphaseBruteForce::removePair(const cvBroadphaseHandle&/* handle1*/, const cvBroadphaseHandle&/* handle2*/)
{
	return false;
}

void cvBroadphaseBruteForce::getAllPairs(std::vector<BPPair>& pairs)
{
	pairs.clear();
}