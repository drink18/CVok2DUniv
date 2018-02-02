#include "cvok2d.h"
#include "cvBroadPhaseSAP.h"
#include <simulation/cvBody.h>

cvBroadphaseSAP::cvBroadphaseSAP(const cvBroadphaseCInfo& cinfo)
	:cvBroadphase(cinfo)
{
	const int initMaxNode = 512;
	for (int i = 0; i < 2; ++i)
	{
		m_EndPoints[i].reserve(initMaxNode * 2);
	}
}

cvBroadphaseHandle cvBroadphaseSAP::addNode(const cvAabb& nodeAabb)
{
	cvBroadphaseHandle handle = m_Nodes.alloc();
	BPNode& newNode = m_Nodes.accessAt(handle);
	newNode.m_aabb = nodeAabb;
	NodeEndPoint min0, min1;
	NodeEndPoint max0, max1;

	min0.m_Val = nodeAabb.m_Min.x; min0.setIsMin(true);
	min1.m_Val = nodeAabb.m_Min.y; min1.setIsMin(true);

	max0.m_Val = nodeAabb.m_Max.x; max0.setIsMin(false);
	max1.m_Val = nodeAabb.m_Max.y; max1.setIsMin(false);

	min0.setBPHandle(handle);
	min1.setBPHandle(handle);
	max0.setBPHandle(handle);
	max1.setBPHandle(handle);

	newNode.m_MinIdx[0] = (int32_t)m_EndPoints[0].size();
	m_EndPoints[0].push_back(min0);
	newNode.m_MaxIdx[0] = (int32_t)m_EndPoints[0].size();
	m_EndPoints[0].push_back(max0);
	newNode.m_MinIdx[1] = (int32_t)m_EndPoints[1].size();
	m_EndPoints[1].push_back(min1);
	newNode.m_MaxIdx[1] = (int32_t)m_EndPoints[1].size();
	m_EndPoints[1].push_back(max1);

    int axis = 0;
    moveEndPoint(axis, newNode.m_MinIdx[axis], min0.m_Val, -1);
    moveEndPoint(axis, newNode.m_MaxIdx[axis], max0.m_Val, -1);
    axis = 1;
    moveEndPoint(axis, newNode.m_MinIdx[axis], min1.m_Val, -1);
    moveEndPoint(axis, newNode.m_MaxIdx[axis], max1.m_Val, -1);

	return handle;
}

void cvBroadphaseSAP::removeNode(cvBroadphaseHandle handle)
{
	BPNode &node = m_Nodes.accessAt(handle);
	for (int axis = 0; axis < 2; ++axis)
	{
		const int minIdx = node.m_MinIdx[axis];
		const int maxIdx = node.m_MaxIdx[axis];
		std::vector<NodeEndPoint>& endPoints = m_EndPoints[axis];
		// removing possible overalpping bp node
		for (int32_t idx = minIdx + 1; idx < endPoints.size(); ++idx)
		{
			NodeEndPoint ep = endPoints[idx];
			if (idx < maxIdx)
			{
				removePair(handle, ep.getBPHandle());
			}

			cvBroadphaseHandle h1 = ep.getBPHandle();
			BPNode& n1 = m_Nodes.accessAt(h1);
			
			// update endpoint indices in bp node
			if (ep.getIsMin())
			{
				n1.m_MinIdx[axis] -= idx < maxIdx ? 1 : 2;
			}
			else
			{
				n1.m_MaxIdx[axis] -= idx < maxIdx ? 1 : 2;
			}
		}

		// clear endpoints
		endPoints.erase(endPoints.begin() + minIdx);
		endPoints.erase(endPoints.begin() + maxIdx - 1);
	}
}

void cvBroadphaseSAP::updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb)
{
	float min[2];
	float max[2];
	min[0] = newAabb.m_Min.x; min[1] = newAabb.m_Min.y;
	max[0] = newAabb.m_Max.x; max[1] = newAabb.m_Max.y;
	
	for (int axis = 0; axis < 2; ++axis)
	{
		updateNodeOnOneAxis(handle.getVal(), min[axis], max[axis], axis);
	}
}

void cvBroadphaseSAP::updateNodeOnOneAxis(int nodeIdx, float min, float max, int axis)
{
	BPNode& dirtyNode = m_Nodes.accessAt(nodeIdx);
	std::vector<NodeEndPoint>& endPointsOneAxis = m_EndPoints[axis];
	NodeEndPoint& oldMinPt = endPointsOneAxis[dirtyNode.m_MinIdx[axis]];
	NodeEndPoint& oldMaxPt = endPointsOneAxis[dirtyNode.m_MaxIdx[axis]];
	enum eDir{Left = -1, Right = 1};

	// determine move direction
	eDir dirMin = min < oldMinPt.m_Val ? Left : Right;
	eDir dirMax= max < oldMaxPt.m_Val ? Left : Right;

	moveEndPoint(axis, dirtyNode.m_MinIdx[axis], min, dirMin);
    endPointsOneAxis[dirtyNode.m_MinIdx[axis]].m_Val = min;

	moveEndPoint(axis, dirtyNode.m_MaxIdx[axis], max, dirMax);
    endPointsOneAxis[dirtyNode.m_MaxIdx[axis]].m_Val = max;
}

void cvBroadphaseSAP::moveEndPoint(int axis, int endPtIdx, float newVal, int direction)
{
	std::vector<NodeEndPoint>& endPoints = m_EndPoints[axis];
	NodeEndPoint ep = endPoints[endPtIdx];
	int endIdx = direction == -1 ? 0 : (int)endPoints.size() - 1;
	for (int i = endPtIdx; i != endIdx; i += direction)
	{
		int nextPt = i + direction;
		NodeEndPoint& nextEp = endPoints[nextPt];
		if ((direction == -1 && nextEp.m_Val > newVal)
			|| (direction == 1 && nextEp.m_Val < newVal))
		{
			if (direction == 1)
			{
				if (nextEp.getIsMin() && ep.getIsMax())
				{
					// add new pair (nextEp, ep)
					addPair(nextEp, ep);
				}
				if (ep.getIsMin() && nextEp.getIsMax())
				{
					// remove pair (nextEp, ep)
					removePair(nextEp, ep);
				}
			}
			else
			{
				if (ep.getIsMin() && nextEp.getIsMax())
				{
					// add new pair (nextExp, ep)
					addPair(nextEp, ep);
				}
				if (ep.getIsMax() && nextEp.getIsMin())
				{
					// remove pair (nextEp, ep)
					removePair(nextEp, ep);
				}
			}
			//swap
			swapEndPoints(i, nextPt, axis);
		}
	}
	
}

void cvBroadphaseSAP::swapEndPoints(int epIdx1, int epIdx2, int axis)
{
	std::vector<NodeEndPoint>& endPoints = m_EndPoints[axis];
	NodeEndPoint& ep1 = endPoints[epIdx1];
	NodeEndPoint& ep2 = endPoints[epIdx2];

	NodeEndPoint tmpNode;

	// update ep index in owner of ep1
	BPNode& node2 = m_Nodes.accessAt(ep2.getBPHandle().getVal());
	const bool isMin2 = ep2.getIsMin();
	int& nextEpIdx2 = isMin2 ? node2.m_MinIdx[axis] : node2.m_MaxIdx[axis];
	nextEpIdx2 = epIdx1;

	BPNode& node1 = m_Nodes.accessAt(ep1.getBPHandle().getVal());
	const bool isMin1 = ep1.getIsMin();
	int& nextEpIdx1 = isMin1 ? node1.m_MinIdx[axis] : node1.m_MaxIdx[axis];
	nextEpIdx1 = epIdx2;

	// swap ep
	tmpNode = ep2;
	ep2 = ep1;
	ep1 = tmpNode;
}

bool cvBroadphaseSAP::addPair(NodeEndPoint& ep1, NodeEndPoint& ep2)
{
	return addPair(ep1.getBPHandle(), ep2.getBPHandle());
}

bool cvBroadphaseSAP::addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2)
{
	BPNode& node1 = m_Nodes.accessAt(handle1);
	BPNode& node2 = m_Nodes.accessAt(handle2);

	if (node1.m_aabb.overlaps(node2.m_aabb))
	{
		BPPair pair(handle1, handle2);
		m_Pairs[pair] = pair;
		return true;
	}
	return false;
}

bool cvBroadphaseSAP::removePair(NodeEndPoint& ep1, NodeEndPoint& ep2)
{
	return removePair(ep1.getBPHandle(), ep2.getBPHandle());
}


bool cvBroadphaseSAP::removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2)
{
	BPPair pair(handle1, handle2);
	if (m_Pairs.find(pair) != m_Pairs.end())
	{
		m_Pairs.erase(pair);
		return true;
	}
	return false;
}

void cvBroadphaseSAP::getAllPairs(std::vector<BPPair>& pairs)
{
	pairs.clear();
	for (auto& end : m_Pairs)
	{
		pairs.push_back(end.first);
	}
}

void cvBroadphaseSAP::addBody(cvBody& body)
{
    cvAabb aabb;
    body.getAabb(aabb);
    body.setBroadphaseHandle(addNode(aabb));
}

void cvBroadphaseSAP::removeBody(cvBody& body)
{
    removeNode(body.getBroadphaseHandle());
    body.setBroadphaseHandle(cvBroadphaseHandle::invalid());
}
