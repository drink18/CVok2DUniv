#pragma once

#include "cvBroadphase.h"

class cvBroadphaseBruteForce : public cvBroadphase
{
	struct BPNode
	{
		cvAabb m_aabb;
	};
public:
	cvBroadphaseBruteForce(cvBroadphaseCInfo& cInfo);

	virtual void updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb) override;
	virtual cvBroadphaseHandle addNode(const cvAabb& nodeAabb) override;
	virtual void removeNode(cvBroadphaseHandle handle) override;
	virtual void getAllPairs(std::vector<BPPair>& pairs) override;
protected:
	virtual bool addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;
	virtual bool removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override;
private:
	cvFreeList<BPNode, cvBroadphaseHandle> m_Nodes;
};