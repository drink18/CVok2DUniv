#pragma once

#include <unordered_set>
#include "cvBroadphase.h"
#include <simulation/cvBody.h>

class cvBroadphaseSAP : public cvBroadphase
{
public:
	struct BPNode
	{
		int m_MinIdx[2];
		int m_MaxIdx[2];
		cvAabb m_aabb;
		int64_t m_userData = 0;
        cvBodyId m_bodyId;

		BPNode()
		{}
	};

	typedef cvBroadphase::NodeEndPoint NodeEndPoint;
	typedef cvBroadphase::BPPair BPPair;
    typedef std::vector<cvBroadphase::NodeEndPoint> NodeEPList;
    typedef cvFreeList<BPNode, cvBroadphaseHandle> NodeList;
    typedef std::unordered_set<BPPair, BPPair::compBPPair > BPPairMap;
public:
	cvBroadphaseSAP(const cvBroadphaseCInfo& cinfo);

    virtual void updateDirtyNodes(std::vector<BPPair>& newPairs, std::vector<BPPair>& removedPairs) override;
	virtual void updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb) override;
	virtual cvBroadphaseHandle addNode(const cvAabb& nodeAabb) override;
	virtual void removeNode(cvBroadphaseHandle handle) override;
	virtual void getAllPairs(std::vector<BPPair>& pairs) override;

    virtual void addBody(cvBody& body) override;
    virtual void removeBody(cvBody& body) override;
    virtual void markBodyDirty(const cvBody& body) override;

	virtual bool addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;
	virtual bool removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;

public:
    const std::unordered_map<cvBroadphaseHandle, cvAabb>& getDirtyNodes() const {return m_DirtyNodes;}
    const NodeEPList& getEpList(int axis) const {return m_EndPoints[axis];}
    const NodeList& getNodeList() const {return m_Nodes;}

	bool addPair(NodeEndPoint& ep1, NodeEndPoint& ep2);
	bool removePair(NodeEndPoint& ep1, NodeEndPoint& ep2);

	static void swapEndPoints(int epIdx1, int epIdx2, NodeEndPoint& ep1, NodeEndPoint& ep2, BPNode& n1, BPNode& n2, int axis);
private:
	void updateNodeOnOneAxis(int nodeIdx, float min, float max, int axis);
	void moveEndPoint(int axis, int endPtIdx, int direction);
private:
	NodeEPList  m_EndPoints[2];
	NodeList m_Nodes;
	BPPairMap m_Pairs;

    std::unordered_map<cvBroadphaseHandle, cvAabb> m_DirtyNodes;
};
