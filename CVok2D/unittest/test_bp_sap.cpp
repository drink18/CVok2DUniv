#include "stdafx.h"

#include "gtest/gtest.h"
#include <cvok2d/cvok2d.h>
#include <cvok2d/simulation/cvBroadphase.h>
#include <cvok2d/simulation/cvBroadPhaseSAP.h>
#include <cvok2d/simulation/cvBody.h>
#include <vector>

TEST(cvBody, body)
{
	cvBodyCInfo bodyCInfo;
	cvBody body(bodyCInfo);
}

TEST(cvBroadphase, addBody)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(0, 0), cvVec2f(1, 1));
	broadPhase.addNode(aabb1);
	
	cvAabb aabb2(cvVec2f(0.5f, 0.5f), cvVec2f(1, 1));
	broadPhase.addNode(aabb2);

	cvAabb aabb3(cvVec2f(-2.5f, -2.5f), cvVec2f(-1.0f, -1.0f));
	broadPhase.addNode(aabb3);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);
}

TEST(cvBroadphase, removeBody)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(0, 0), cvVec2f(1, 1));
	broadPhase.addNode(aabb1);
	
	cvAabb aabb2(cvVec2f(0.5f, 0.5f), cvVec2f(1, 1));
    cvBroadphaseHandle handle2 = broadPhase.addNode(aabb2);

	cvAabb aabb3(cvVec2f(-2.5f, -2.5f), cvVec2f(-1.0f, -1.0f));
	cvBroadphaseHandle handle3 = broadPhase.addNode(aabb3);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);

	broadPhase.removeNode(handle3);

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);

	broadPhase.removeNode(handle2);

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 0);
}
