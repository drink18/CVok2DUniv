#pragma once

#include "cvBroadphase.h"

class cvBroadphaseSAP : public cvBroadphase
{
	typedef cvBroadphase::NodeEndPoint NodeEndPoint;
	typedef cvBroadphase::BPPair BPPair;

	struct BPNode
	{
		int m_MinIdx[2];
		int m_MaxIdx[2];
		cvAabb m_aabb;
		int64_t m_userData;

		BPNode()
			:m_userData(0)
		{}
	};
public:
	cvBroadphaseSAP(const cvBroadphaseCInfo& cinfo);

	virtual void updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb) override;
	virtual cvBroadphaseHandle addNode(const cvAabb& nodeAabb) override;
	virtual void removeNode(cvBroadphaseHandle handle) override;
	virtual void getAllPairs(std::vector<BPPair>& pairs) override;
    virtual void addBody(cvBody& body) override;
protected:
    virtual void removeBody(cvBody& body) override;

	bool addPair(NodeEndPoint& ep1, NodeEndPoint& ep2);
	bool removePair(NodeEndPoint& ep1, NodeEndPoint& ep2);
	
	virtual bool addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;
	virtual bool removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;
private:
	void updateNodeOnOneAxis(int nodeIdx, float min, float max, int axis);
	void swapEndPoints(int epIdx1, int epIdx2, int axis);
	void moveEndPoint(int axis, int endPtIdx, int direction);
private:
	std::vector<cvBroadphase::NodeEndPoint>  m_EndPoints[2];
	cvFreeList<BPNode, cvBroadphaseHandle> m_Nodes;
	std::unordered_map<BPPair, BPPair,BPPair::compBPPair > m_Pairs;
};
