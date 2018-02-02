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

TEST(cvBroadphase, updateBody_MoveRight)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(0, 0), cvVec2f(1, 1));
	auto handle1 = broadPhase.addNode(aabb1);

	cvAabb aabb2(cvVec2f(1.5f, 1.5f), cvVec2f(3, 3));
    cvBroadphaseHandle handle2 = broadPhase.addNode(aabb2);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 0);

    broadPhase.updateOneNode(handle1, cvAabb(cvVec2f(2.0f, 2.0f), cvVec2f(3.0f, 3.f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);
}

TEST(cvBroadphase, updateBody_MoveLeft)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(2.0f, 2.0f), cvVec2f(3.0f, 3.0f));
	auto handle1 = broadPhase.addNode(aabb1);

	cvAabb aabb2(cvVec2f(1.5f, 1.5f), cvVec2f(3, 3));
    cvBroadphaseHandle handle2 = broadPhase.addNode(aabb2);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);

    broadPhase.updateOneNode(handle1, cvAabb(cvVec2f(0, 0), cvVec2f(1.0f, 1.0f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 0);
}

TEST(cvBroadphase, updateBody_MoveExpand)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(-2.0f, -2.0f), cvVec2f(-1.0f, -1.0f));
	auto handle1 = broadPhase.addNode(aabb1);

	cvAabb aabb2(cvVec2f(1.0f, 1.0f), cvVec2f(2, 2));
    cvBroadphaseHandle handle2 = broadPhase.addNode(aabb2);


	cvAabb aabb3(cvVec2f(-0.9f, -0.9f), cvVec2f(0.9f, 0.9f));
    cvBroadphaseHandle handle3 = broadPhase.addNode(aabb3);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 0);

    broadPhase.updateOneNode(handle3, cvAabb(cvVec2f(-1.5f, -1.5f), cvVec2f(1.5f, 1.5f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 2);
}
