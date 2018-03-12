#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <cvok2d/cvok2d.h>
#include <cvok2d/simulation/cvBroadphase.h>
#include <cvok2d/simulation/cvBroadPhaseSAP.h>
#include <cvok2d/simulation/cvBody.h>
#include <cvok2d/simulation/cvWorld.h>
#include <cvok2d/simulation/cvSimulationContext.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/simulation/cvSimulationControlSimple.h>

#include <memory>
#include <vector>

class FakeBP :public cvBroadphase
{
public:
    FakeBP(cvBroadphaseCInfo cinfo) :handle1(0), handle2(1)
    {
        m_node1.m_bodyId = cvBodyId(0);
        m_node2.m_bodyId = cvBodyId(0);
    }

    virtual void updateDirtyNodes(std::vector<BPPair>& newPairs, std::vector<BPPair>& removedPairs)  override
    {
        BPPair pair(handle1, handle2);
        if(m_adding)
        {
            newPairs.push_back(pair);
        }
        else
        {
            removedPairs.push_back(pair);
        }
    }

	virtual void updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb) override
    {
    }

	virtual cvBroadphaseHandle addNode(const cvAabb& nodeAabb) override
    {
        return handle1;
    }

	virtual void removeNode(cvBroadphaseHandle handle) override { }

	virtual void getAllPairs(std::vector<BPPair>& pairs) override { }

    virtual void addBody(cvBody& body) override { }

    virtual void removeBody(cvBody& body) override { }

    virtual void markBodyDirty(const cvBody& body) override { }


	virtual bool addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override
    {
        return true;
    }

	virtual bool removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) override
    {
        return true;
    }

    virtual const BPNodeBase* getBPNode(cvBroadphaseHandle handle) const  override
    {
        if(handle == handle1) return &m_node1;
        return &m_node2;
    }

    bool m_adding = true; // when false, updateDirtyNode will mark fake pair as removed

    cvBroadphaseHandle handle1;
    cvBroadphaseHandle handle2;
    BPNodeBase m_node1;
    BPNodeBase m_node2;
};

using namespace std;
class TestSimControl : public ::testing::Test
{
public:
    virtual void SetUp() override
    {
        cvBroadphaseCInfo bpInfo;
        m_bp = new FakeBP(bpInfo);

        cvWorldCInfo wldInfo;
        wldInfo.m_broadPhase = m_bp;
        m_world = new cvWorld(wldInfo);

        cvBodyCInfo info;
        info.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
        m_bodyId = m_world->createBody(info, true);

        m_ctx = new cvSimulationContext();

        m_sc.reset(new cvSimulationControlSimple(m_bp, m_ctx, m_world));
    }
    unique_ptr<cvSimulationControlSimple> m_sc;
    cvSimulationContext* m_ctx;

    FakeBP* m_bp = nullptr;
    cvWorld* m_world = nullptr;
    cvBodyId m_bodyId;
};


TEST_F(TestSimControl, generateColAgent)
{
    m_sc->updateBP();

    EXPECT_EQ(1, m_ctx->m_colAgents.size());
}

TEST_F(TestSimControl, removeColAgent)
{
    m_sc->updateBP();
    m_bp->m_adding = false;
    m_sc->updateBP();

    EXPECT_EQ(0, m_ctx->m_colAgents.size());
}
