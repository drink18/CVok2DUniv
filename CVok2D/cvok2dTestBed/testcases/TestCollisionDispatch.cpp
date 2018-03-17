#include "TestCollisionDispatch.h"
#include <world/cvWorld.h>
#include <world/cvBroadPhaseSAP.h>
#include <shape/cvCircle.h>
#include "../DebugDraw.h"

TestCollisionDispatch::TestCollisionDispatch()
{
    cvWorldCInfo info;
    cvBroadphaseCInfo bpInfo;
    info.m_broadPhase = new cvBroadphaseSAP(bpInfo);
    m_world.reset(new cvWorld(info));

    {
        cvCircle* circle = new cvCircle(cvVec2f(0.0f, 0.0f), 1.0f);
        cvBodyCInfo binfo;
        binfo.m_initTransform.m_Translation.set(1.05f, 0.0f);
        binfo.m_shape.reset(circle);
        m_world->createBody(binfo, true);
    }

    {
        cvCircle* circle = new cvCircle(cvVec2f(0.0f, 0.0f), 1.0f);
        cvBodyCInfo binfo;
        binfo.m_initTransform.m_Translation.set(-1.05f, 0.0f);
        binfo.m_shape.reset(circle);
        m_world->createBody(binfo, true);
    }


}

void TestCollisionDispatch::tick(cvDebugDraw& dbgDraw, float dt)
{
    m_world->integrate(dt);

    dbgDraw.DrawWorld(*m_world);
}

REGISTER_TEST("TestCollisionDispatch", [](){return new TestCollisionDispatch();});
