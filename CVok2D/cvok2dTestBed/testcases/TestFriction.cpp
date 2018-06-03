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
    m_shape = shared_ptr<cvPolygonShape>(
        cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));

    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);

        bodyInfo.m_shape = shared_ptr<cvPolygonShape>(
            cvPolygonShape::createBox(cvVec2f(40.0f, 0.5f), 0.05f));

        auto id = m_world->createBody(bodyInfo, true);

    }

    // boxes
    float hf = 1.3f;
    for (int i = 0; i < 10; ++i)
    {
        for (int j = 0; j < 1; ++j)
        {
            //auto shape = shared_ptr<cvPolygonShape>(
                //cvPolygonShape::createBox(cvVec2f(hf, hf), 0.05f));
            auto shape = make_shared<cvCircle>(cvVec2f(0, 0), hf);
            cvBodyCInfo bodyInfo;
            bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f + 2.2f * j,
                (2 * hf) * i - 4.5f);
            //bodyInfo.m_initTransform.m_Rotation = 0.1f;
            bodyInfo.m_mass = 1.0f + i;
            bodyInfo.m_shape = shape;

            auto id = m_world->createBody(bodyInfo, true);
            //m_world->setBodyLinearVelocity(id, cvVec2f(-10.5f, 0));
        }
    }

    for (int i = 0; i < 0; ++i)
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(-17.0f + i * 0.1f, i * 2.0f);
        bodyInfo.m_mass = 30.0f;
        bodyInfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 0.5f);

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

REGISTER_TEST("TestFriction", []() {return new TestFriction(); });
