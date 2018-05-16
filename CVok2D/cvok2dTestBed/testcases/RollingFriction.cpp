#include "TestBase.h"
#include <world/cvBody.h>
#include <memory>

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCompoundShape.h>
#include <world/cvWorld.h>
#include <world/cvMaterial.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

class RollingFrictionTest: public TestBase
{
public:
     RollingFrictionTest();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    cvWorld* m_world = nullptr;
};

static void MakeTestSlopeAndObj(float y, cvWorld* world, float fric, float rollFric)
{
    auto platMat = make_shared<cvMaterial>();
    auto bodyMat = make_shared<cvMaterial>();
    platMat->m_friction = 1.0f;
    platMat->m_rollingFriction = 0.0f;
    bodyMat->m_friction = fric;
    bodyMat->m_rollingFriction = rollFric;

    {
        cvBodyCInfo cinfo;
        cinfo.m_shape = shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(20, 0.5f), 0.1f));
        cinfo.m_initTransform.m_Translation = cvVec2f(0, y);
        cinfo.m_material = platMat;
        cinfo.m_motionType = cvMotion::MotionType::Static;
        world->createBody(cinfo, true);

        cinfo.m_shape = shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(5.0f, 0.5f), 0.1f));
        cinfo.m_initTransform.m_Translation = cvVec2f(-10.0f, y + 3);
        cinfo.m_initTransform.m_Rotation = DEG2RAD(-45);
        cinfo.m_material = platMat;
        world->createBody(cinfo, true);


        cinfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 0.5f);
        cinfo.m_motionType = cvMotion::MotionType::Dynamic;
        cinfo.m_initTransform.m_Translation = cvVec2f(-12.0f, 7 + y);
        cinfo.m_material = bodyMat;
        world->createBody(cinfo, true);
    }
}

RollingFrictionTest::RollingFrictionTest()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    MakeTestSlopeAndObj(-20.0f, m_world, 0.8f, 0.2f);
    MakeTestSlopeAndObj(-10.0f, m_world, 0.6f, 0.1f);
    MakeTestSlopeAndObj(0.0f, m_world, 0.2f, 0.05f);
}

void RollingFrictionTest::tick(cvDebugDraw& debugDraw, float dt)
{
    cvSimInfo simInfo;
    simInfo.deltaTime = dt;
    m_world->simulate(simInfo);
    debugDraw.DrawWorld(*m_world);
}

REGISTER_TEST("RollingFrictionTest", [](){return new RollingFrictionTest();});
