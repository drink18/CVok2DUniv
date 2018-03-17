#include "cvok2d.h"
#include "cvBroadPhaseSAP.h"
#include <world/cvBody.h>
#include <world/cvWorld.h>

cvBroadphaseSAP::cvBroadphaseSAP(const cvBroadphaseCInfo& cinfo)
	:cvBroadphase(cinfo)
{
	const int initMaxNode = 512;
	for (int i = 0; i < 2; ++i)
	{
		m_EndPoints[i].reserve(initMaxNode * 2);
	}

    m_AABBExpansion = cinfo.m_AABBExpansion;
}

cvBroadphaseHandle cvBroadphaseSAP::addNode(const cvAabb& nodeAabb)
{
    cvVec2f he = nodeAabb.m_Max - nodeAabb.m_Min;
    cvAssertMsg(he.x > CV_FLOAT_EPS && he.y > CV_FLOAT_EPS, "degenerated aabb");

	cvBroadphaseHandle handle = m_Nodes.alloc();
	BPNode& newNode = m_Nodes.accessAt(handle);
	newNode.m_aabb = nodeAabb;
	NodeEndPoint min0, min1;
	NodeEndPoint max0, max1;

	min0.m_Val = FLT_MAX; min0.setIsMin(true);
	min1.m_Val = FLT_MAX; min1.setIsMin(true);

	max0.m_Val = FLT_MAX; max0.setIsMin(false);
	max1.m_Val = FLT_MAX; max1.setIsMin(false);

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

    m_DirtyNodes[handle] = nodeAabb;

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
			if (
                    ep.getIsMin())
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

	BPNode& dirtyNode = m_Nodes.accessAt(handle.getVal());
	dirtyNode.m_aabb = newAabb;

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

	endPointsOneAxis[dirtyNode.m_MinIdx[axis]].m_Val = min;
    endPointsOneAxis[dirtyNode.m_MaxIdx[axis]].m_Val = max;

    int minIdx = dirtyNode.m_MinIdx[axis];
    int maxIdx = dirtyNode.m_MaxIdx[axis];
	moveEndPoint(axis, minIdx, dirMin);
	moveEndPoint(axis, maxIdx, dirMax);
}

void cvBroadphaseSAP::moveEndPoint(int axis, int endPtIdx, int direction)
{
	std::vector<NodeEndPoint>& endPoints = m_EndPoints[axis];
	int endIdx = direction == -1 ? 0 : (int)endPoints.size() - 1;
	for (int i = endPtIdx; i != endIdx; i += direction)
	{
		NodeEndPoint& ep = endPoints[i];
		int nextPt = i + direction;
		NodeEndPoint& nextEp = endPoints[nextPt];



		if (((direction == -1 && nextEp.m_Val > ep.m_Val)
			|| (direction == 1 && nextEp.m_Val < ep.m_Val)
            ) )
		{
            // newly added pairs has FLT_MAX as aabb min and max, skip those
            if(nextEp.m_Val != FLT_MAX && ep.m_Val != FLT_MAX)
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
            }

            //swap
            swapEndPoints(i, nextPt, ep, nextEp, m_Nodes.accessAt(ep.getBPHandle()), 
                   m_Nodes.accessAt(nextEp.getBPHandle()), axis);
		}
	}
}

void cvBroadphaseSAP::swapEndPoints(int epIdx1, int epIdx2, NodeEndPoint& ep1, NodeEndPoint& ep2, 
        BPNode& node1, BPNode& node2, int axis)
{
	NodeEndPoint tmpNode;

	// update ep index in owner of ep1
	const bool isMin2 = ep2.getIsMin();
	int& nextEpIdx2 = isMin2 ? node2.m_MinIdx[axis] : node2.m_MaxIdx[axis];
	nextEpIdx2 = epIdx1;

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
		m_Pairs.insert(pair);
        m_newPairs.insert(pair);
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
        m_removedPairs.insert(pair);
		return true;
	}
	return false;
}

void cvBroadphaseSAP::getAllPairs(std::vector<BPPair>& pairs)
{
	pairs.clear();
	for (auto& end : m_Pairs)
	{
		pairs.push_back(end);
	}
}

void cvBroadphaseSAP::addBody(cvBody& body)
{
    cvAabb aabb;
    body.getAabb(aabb);
    cvVec2f exp = cvVec2f(m_AABBExpansion, m_AABBExpansion);
    aabb.expand(exp);

    cvBroadphaseHandle bpHandle = addNode(aabb);
    body.setBroadphaseHandle(bpHandle);
    m_Nodes.accessAt(bpHandle).m_bodyId = body.getBodyId();
}

void cvBroadphaseSAP::removeBody(cvBody& body)
{
    removeNode(body.getBroadphaseHandle());
    body.setBroadphaseHandle(cvBroadphaseHandle::invalid());
}

void cvBroadphaseSAP::markBodyDirty(const cvBody& body)
{
    auto bphandle = body.getBroadphaseHandle();
    cvAssertMsg(bphandle.isValid(), "Marking a invalid bp node dirty");
    cvAabb aabb;
    body.getAabb(aabb);
    cvVec2f exp = cvVec2f(m_AABBExpansion, m_AABBExpansion);
    aabb.expand(exp);
    m_DirtyNodes[bphandle] = aabb;
}

void cvBroadphaseSAP::updateDirtyNodes(std::vector<BPPair>& newPairs, std::vector<BPPair>& removedPairs)
{
    m_removedPairs.clear();
    m_newPairs.clear();

    newPairs.clear();
    removedPairs.clear();

    for(auto& dn: m_DirtyNodes)
    {
        const BPNode& node = m_Nodes.getAt(dn.first);
        updateOneNode(dn.first, dn.second);
    }

    for(auto& n : m_newPairs)
    {
        if(m_removedPairs.find(n) != m_removedPairs.cend())
        {
            m_removedPairs.erase(n);
        }
        else
        {
            newPairs.push_back(n);
        }
    }

    for(auto& n : m_removedPairs)
        removedPairs.push_back(n);

    m_DirtyNodes.clear();
}

void cvBroadphaseSAP::getBpAABB(cvBroadphaseHandle handle, cvAabb& outAabb) const
{
    outAabb = m_Nodes.getAt(handle).m_aabb;
}
