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

using namespace std;
class TestSimControl : public ::testing::Test
{
public:
    virtual void SetUp() override
    {
        cvBroadphaseCInfo bpInfo;
        m_bp = new cvBroadphaseSAP(bpInfo);

        cvWorldCInfo wldInfo;
        wldInfo.m_broadPhase = m_bp;
        m_world = new cvWorld(wldInfo);

        cvBodyCInfo info;
        info.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
        m_bodyId = m_world->createBody(info, true);

        m_ctx = new cvSimulationContext();

        m_sc.reset(new cvSimulationControlSimple(m_bp, m_ctx));
    }
    unique_ptr<cvSimulationControlSimple> m_sc;
    cvSimulationContext* m_ctx;

    cvBroadphaseSAP* m_bp = nullptr;
    cvWorld* m_world = nullptr;
    cvBodyId m_bodyId;
};


TEST_F(TestSimControl, generateColAgent)
{
    m_sc->updateBP();

    EXPECT_EQ(1, m_ctx->m_colAgents.size());
}
