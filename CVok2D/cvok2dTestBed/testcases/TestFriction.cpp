#include "TestFriction.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <world/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

TestFriction::TestFriction()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    m_shape = shared_ptr<cvPolygonShape> (
            cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));

    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);

        bodyInfo.m_shape = shared_ptr<cvPolygonShape> (
                cvPolygonShape::createBox(cvVec2f(40.0f, 0.5f), 0.05f));

        auto id = m_world->createBody(bodyInfo, true);

    }

    // boxes
    for(int i = 0; i < 15; ++i)
    {
        for(int j = 0; j < 1; ++j)
        {
            cvBodyCInfo bodyInfo;
            bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f + 2.2f * j, 
                    2.2f * i - 4.5f);
            bodyInfo.m_mass = 1.0f;
            bodyInfo.m_shape = m_shape;

            auto id = m_world->createBody(bodyInfo, true);
            //m_world->setBodyLinearVelocity(id, cvVec2f(-10.5f, 0));
        }
    }

    for(int i = 0; i < 0; ++i)
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(-17.0f + i * 0.1f, i * 2.0f);
        bodyInfo.m_mass = 30.0f;
        bodyInfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);

        auto id = m_world->createBody(bodyInfo, true);
        m_world->setBodyLinearVelocity(id, cvVec2f(-0.5f, 0));
        //m_world->setBodyAngularVelocity(m_Id, 0.0f);
    }
}

void TestFriction::tick(cvDebugDraw& debugDraw, float dt)
{
    cvSimInfo simInfo;
    simInfo.deltaTime = dt;
    m_world->simulate(simInfo);
    debugDraw.DrawWorld(*m_world);
}

REGISTER_TEST("TestFriction", [](){return new TestFriction();});
