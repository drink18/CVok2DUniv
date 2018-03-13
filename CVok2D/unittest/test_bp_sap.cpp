#include "gtest/gtest.h"
#include <cvok2d/cvok2d.h>
#include <cvok2d/world/cvBroadphase.h>
#include <cvok2d/world/cvBroadPhaseSAP.h>
#include <cvok2d/world/cvBody.h>
#include <vector>

using namespace std;

cvBroadphaseHandle add_node_helper(cvAabb aabb, cvBroadphase* bp)
{
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> removedPairs;

	auto handle = bp->addNode(aabb);
    bp->updateDirtyNodes(newPairs, removedPairs);
    return handle;
}

class TestBroadphaseSAP: public ::testing::Test
{
public:
    virtual void SetUp()
    {
        auto h1 = nodeList.alloc();
        auto& n1 = nodeList.accessAt(h1);
        n1.m_MinIdx[0] = 1;
        n1.m_MinIdx[1] = 1;
        n1.m_MaxIdx[0] = 2;
        n1.m_MaxIdx[1] = 2;

        auto h2 = nodeList.alloc();
        auto& n2 = nodeList.accessAt(h2);
        n2.m_MinIdx[0] = 3;
        n2.m_MinIdx[1] = 3;
        n2.m_MaxIdx[0] = 4;
        n2.m_MaxIdx[1] = 4;

        cvBroadphase::NodeEndPoint ep1;
        ep1.m_NodeHandleAndMinFlag = 0;
        ep1.m_Val = 1.0f;
        ep1.setIsMin(true);

        cvBroadphase::NodeEndPoint ep2;
        ep2.m_NodeHandleAndMinFlag = 1;
        ep2.m_Val = 2.0f;
        ep2.setIsMin(true);

        epList.push_back(ep1);
        epList.push_back(ep2);
    }


    cvBroadphaseSAP::NodeList nodeList;
    cvBroadphaseSAP::NodeEPList epList;
};

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
    add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(0.5f, 0.5f), cvVec2f(1, 1));
    add_node_helper(aabb2, &broadPhase);

	cvAabb aabb3(cvVec2f(-2.5f, -2.5f), cvVec2f(-1.0f, -1.0f));
    add_node_helper(aabb3, &broadPhase);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(pairs.size(), 1);
}

TEST(cvBroadphase, removeBody)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(0, 0), cvVec2f(1, 1));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(0.5f, 0.5f), cvVec2f(1, 1));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

	cvAabb aabb3(cvVec2f(-2.5f, -2.5f), cvVec2f(-1.0f, -1.0f));
    auto handle3 = add_node_helper(aabb3, &broadPhase);


	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(1, pairs.size());

	broadPhase.removeNode(handle3);

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(1, pairs.size());

	broadPhase.removeNode(handle2);
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(0, pairs.size());
}

TEST(cvBroadphase, updateBody_MoveRight)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(0, 0), cvVec2f(1, 1));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(1.5f, 1.5f), cvVec2f(3, 3));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(0, pairs.size());

    broadPhase.updateOneNode(handle1, cvAabb(cvVec2f(2.0f, 2.0f), cvVec2f(3.0f, 3.f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(1, pairs.size());
}

TEST(cvBroadphase, updateBody_MoveLeft)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(2.0f, 2.0f), cvVec2f(3.0f, 3.0f));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(1.5f, 1.5f), cvVec2f(3, 3));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(1, pairs.size());

    broadPhase.updateOneNode(handle1, cvAabb(cvVec2f(0, 0), cvVec2f(1.0f, 1.0f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(0, pairs.size());
}

TEST(cvBroadphase, updateBody_MoveExpand)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	cvAabb aabb1(cvVec2f(-2.0f, -1.0f), cvVec2f(-1.0f, 1.0f));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(1.0f, -1.0f), cvVec2f(2, 1));
    auto handle2 = add_node_helper(aabb2, &broadPhase);


	cvAabb aabb3(cvVec2f(-0.9f, -1.0f), cvVec2f(0.9f, 1.0f));
    auto handle3 = add_node_helper(aabb3, &broadPhase);

	std::vector<cvBroadphase::BPPair> pairs;
	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(0, pairs.size());

    broadPhase.updateOneNode(handle3, cvAabb(cvVec2f(-1.5f, -1.0f), cvVec2f(1.5f, 1.0f)));

	broadPhase.getAllPairs(pairs);
	EXPECT_EQ(2, pairs.size());
}

TEST(cvBroadphase, addNode_testEP)
{
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> removedPairs;
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);

	cvAabb aabb1(cvVec2f(1, 1), cvVec2f(2, 2));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

    const cvBroadphaseSAP::NodeEPList& epList = broadPhase.getEpList(0);

	cvAabb aabb2(cvVec2f(3, 3), cvVec2f(4, 4));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

    broadPhase.updateDirtyNodes(newPairs, removedPairs);

    EXPECT_EQ(4, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(2, epList[1].m_Val);
    EXPECT_EQ(3, epList[2].m_Val);
    EXPECT_EQ(4, epList[3].m_Val);
}

TEST(cvBroadphase, batch_adding)
{
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> removedPairs;
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);

	cvAabb aabb1(cvVec2f(1, 1), cvVec2f(2, 2));
    //auto handle1 = add_node_helper(aabb1, &broadPhase);
    auto handle1 = broadPhase.addNode(aabb1);

    const cvBroadphaseSAP::NodeEPList& epList = broadPhase.getEpList(0);

	cvAabb aabb2(cvVec2f(3, 3), cvVec2f(4, 4));
    //auto handle2 = add_node_helper(aabb2, &broadPhase);
    auto handle2 = broadPhase.addNode(aabb2);

    broadPhase.updateDirtyNodes(newPairs, removedPairs);

    EXPECT_EQ(4, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(2, epList[1].m_Val);
    EXPECT_EQ(3, epList[2].m_Val);
    EXPECT_EQ(4, epList[3].m_Val);
}

TEST(cvBroadphase, remove_node)
{
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> removedPairs;
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);

	cvAabb aabb1(cvVec2f(1, 1), cvVec2f(2, 2));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(3, 3), cvVec2f(4, 4));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

    const cvBroadphaseSAP::NodeEPList& epList = broadPhase.getEpList(0);
    EXPECT_EQ(4, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(2, epList[1].m_Val);
    EXPECT_EQ(3, epList[2].m_Val);
    EXPECT_EQ(4, epList[3].m_Val);

    auto& node1 = broadPhase.getNodeList().getAt(handle1);
    auto& node2 = broadPhase.getNodeList().getAt(handle2);

    EXPECT_EQ(0, node1.m_MinIdx[0]);
    EXPECT_EQ(0, node1.m_MinIdx[1]);
    EXPECT_EQ(1, node1.m_MaxIdx[0]);
    EXPECT_EQ(1, node1.m_MaxIdx[1]);

    EXPECT_EQ(2, node2.m_MinIdx[0]);
    EXPECT_EQ(2, node2.m_MinIdx[1]);

    broadPhase.removeNode(handle1);

    EXPECT_EQ(0, node2.m_MinIdx[0]);
    EXPECT_EQ(0, node2.m_MinIdx[1]);
}

TEST(cvBroadphase, addOverlappingNode_OnRight_testEP)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);

	cvAabb aabb1(cvVec2f(1, 1), cvVec2f(2, 2));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

    const cvBroadphaseSAP::NodeEPList& epList = broadPhase.getEpList(0);
    EXPECT_EQ(2, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(2, epList[1].m_Val);

	cvAabb aabb2(cvVec2f(1.5f, 1.5f), cvVec2f(4, 4));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

    EXPECT_EQ(4, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(1.5f, epList[1].m_Val);
    EXPECT_EQ(2, epList[2].m_Val);
    EXPECT_EQ(4, epList[3].m_Val);
}

TEST(cvBroadphase, addOverlappingNode_OnLeft_testEP)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	std::vector<cvBroadphase::BPPair> pairs;

	cvAabb aabb1(cvVec2f(1, 0), cvVec2f(2, 1));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

    const cvBroadphaseSAP::NodeEPList& epList = broadPhase.getEpList(0);
    EXPECT_EQ(2, epList.size());
    EXPECT_EQ(1, epList[0].m_Val);
    EXPECT_EQ(2, epList[1].m_Val);

	cvAabb aabb2(cvVec2f(0, 0), cvVec2f(1.5f, 1));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

    EXPECT_EQ(4, epList.size());
    EXPECT_EQ(0, epList[0].m_Val);
    EXPECT_EQ(1, epList[1].m_Val);
    EXPECT_EQ(1.5f, epList[2].m_Val);
    EXPECT_EQ(2, epList[3].m_Val);

    broadPhase.getAllPairs(pairs);
    EXPECT_EQ(1, pairs.size());
}

TEST(cvBroadphase, add_degenerated_node )
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	std::vector<cvBroadphase::BPPair> pairs;

	cvAabb aabb1(cvVec2f(1, 0), cvVec2f(2, 0));
	ASSERT_DEATH({broadPhase.addNode(aabb1);}, ".*he.x > CV_FLOAT_EPS && he.y > CV_FLOAT_EPS.*");

	cvAabb aabb2(cvVec2f(0, 0), cvVec2f(0, 1));
	ASSERT_DEATH({broadPhase.addNode(aabb2);}, ".*he.x > CV_FLOAT_EPS && he.y > CV_FLOAT_EPS.*");
}

TEST(cvBroadphase,  add_exact_boundingVolume)
{
	cvBroadphaseCInfo cInfo;
	cvBroadphaseSAP broadPhase(cInfo);
	std::vector<cvBroadphase::BPPair> pairs;

	cvAabb aabb1(cvVec2f(1, -1), cvVec2f(2, 1));
    auto handle1 = add_node_helper(aabb1, &broadPhase);

	cvAabb aabb2(cvVec2f(1, -1), cvVec2f(2, 1));
    auto handle2 = add_node_helper(aabb2, &broadPhase);

    broadPhase.getAllPairs(pairs);
    EXPECT_EQ(1, pairs.size());
}


