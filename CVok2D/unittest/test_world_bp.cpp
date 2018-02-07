#include "gtest/gtest.h"
#include <cvok2d/cvok2d.h>
#include <cvok2d/simulation/cvBroadphase.h>
#include <cvok2d/simulation/cvBroadPhaseSAP.h>
#include <cvok2d/simulation/cvBody.h>
#include <cvok2d/simulation/cvWorld.h>
#include <cvok2d/shape/cvCircle.h>

#include <vector>
#include <memory>

using namespace std;
class TestWorldAndBP : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        cvBroadphaseCInfo bpInfo;
        m_bp = new cvBroadphaseSAP(bpInfo);

        cvWorldCInfo wldInfo;
        wldInfo.m_broadPhase = m_bp;
        m_world = new cvWorld(wldInfo);

        cvBodyCInfo info;
        info.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
        m_bodyId = m_world->createBody(info, true);
    }

    virtual void TearDown()
    {
        delete m_world;
    }

    cvBroadphaseSAP* m_bp = nullptr;
    cvWorld* m_world = nullptr;
    cvBodyId m_bodyId;
};

TEST_F(TestWorldAndBP, BPNodeHasValidBodyId)
{
    cvBody& body = m_world->accessBody(m_bodyId);
    cvBroadphaseHandle bpHandle = body.getBroadphaseHandle();
    EXPECT_TRUE(bpHandle.isValid());

    auto& bpNode = m_bp->getNodeList().getAt(bpHandle);
    EXPECT_TRUE(bpNode.m_bodyId.isValid());
}

TEST_F(TestWorldAndBP, markBodyDirty)
{
    m_bp->markBodyDirty(m_world->accessBody(m_bodyId));
    EXPECT_EQ(1, m_bp->getDirtyNodes().size());
}

TEST_F(TestWorldAndBP, updateDirtyBP)
{
    m_bp->markBodyDirty(m_world->accessBody(m_bodyId));
    EXPECT_EQ(1, m_bp->getDirtyNodes().size());

    m_bp->updateDirtyNodes(*m_world);
    EXPECT_EQ(0, m_bp->getDirtyNodes().size());
}
