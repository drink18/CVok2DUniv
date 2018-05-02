#include "Stacks.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <world/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

Stacks::Stacks()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    m_shape = shared_ptr<cvPolygonShape> (
            cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));

    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(10.0f, 0.0f);
        bodyInfo.m_initTransform.m_Rotation = DEG2RAD(40);
        bodyInfo.m_mass = 1.0f;
        bodyInfo.m_shape = m_shape;

        auto id = m_world->createBody(bodyInfo, true);
        m_world->setBodyAngularVelocity(m_Id, 0.0f);
    }

    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, 0.0f);
        bodyInfo.m_initTransform.m_Rotation = DEG2RAD(45);
        bodyInfo.m_mass = 1.0f;
        bodyInfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);

        m_Id = m_world->createBody(bodyInfo, true);
        //m_world->setBodyAngularVelocity(m_Id, 0.0f);
    }


    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);

        bodyInfo.m_shape = shared_ptr<cvPolygonShape> (
                cvPolygonShape::createBox(cvVec2f(20.0f, 0.5f), 0.05f));

        auto id = m_world->createBody(bodyInfo, true);
    }
}

void Stacks::tick(cvDebugDraw& debugDraw, float dt)
{
    cvSimInfo simInfo;
    simInfo.deltaTime = dt;
    m_world->simulate(simInfo);
    debugDraw.DrawWorld(*m_world);
}

REGISTER_TEST("Stacks", [](){return new Stacks();});
