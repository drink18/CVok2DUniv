#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <cvok2d/cvok2d.h>
#include <cvok2d/world/cvBroadphase.h>
#include <cvok2d/world/cvBroadPhaseSAP.h>
#include <cvok2d/world/cvBody.h>
#include <cvok2d/world/cvWorld.h>
#include <cvok2d/simulation/cvSimulationContext.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/simulation/cvSimulationControlSimple.h>

using namespace std;
class NPTest : public ::testing::Test
{
    virtual void SetUp() override
    {
        cvBroadphaseCInfo bpInfo;
        cvBroadphaseSAP* sap = new cvBroadphaseSAP(bpInfo);

        cvWorldCInfo wldInfo;
        wldInfo.m_broadPhase = sap;
        m_world.reset(new cvWorld(wldInfo));
        m_ctx = new cvSimulationContext() ;
        m_sc.reset(new cvSimulationControlSimple(sap, m_ctx, m_world.get()));

        // add bodies
        cvBodyCInfo info;
        info.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
        m_world->createBody(info, true);

        info.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
        info.m_initTransform.m_Translation = cvVec2f(0.7f, 0);
        m_world->createBody(info, true);
    }

public:
    unique_ptr<cvWorld> m_world;
    unique_ptr<cvSimulationControlSimple> m_sc;
    cvSimulationContext* m_ctx;
};

TEST_F(NPTest, testNPPair)
{
    m_sc->updateBP();
    EXPECT_EQ(1, m_ctx->m_NpPairs.size());

}
