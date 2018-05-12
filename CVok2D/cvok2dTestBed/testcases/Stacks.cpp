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
    //cInfo.m_bpAABBExpesnion = 0.2f;
    //cInfo.m_gravity = cvVec2f::getZero();
    m_world = new cvWorld(cInfo);

    m_shape = shared_ptr<cvPolygonShape> (
            cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));

    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);

        bodyInfo.m_shape = shared_ptr<cvPolygonShape> (
                cvPolygonShape::createBox(cvVec2f(20.0f, 0.5f), 0.05f));

        auto id = m_world->createBody(bodyInfo, true);

        //slop
        bodyInfo.m_initTransform.m_Translation.set(-15, -15); 
        bodyInfo.m_initTransform.m_Rotation = -45;
        m_world->createBody(bodyInfo, true);

        bodyInfo.m_initTransform.m_Translation.set(25, -15); 
        bodyInfo.m_initTransform.m_Rotation = 45;
        m_world->createBody(bodyInfo, true);
    }

    // boxes
    for(int i = 0; i < 8; ++i)
    {
        for(int j = 0; j < 5; ++j)
        {
            cvBodyCInfo bodyInfo;
            bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f + 2.1f * j, 2.1f * i - 8.5f);
            bodyInfo.m_mass = 1.0f;
            bodyInfo.m_shape = m_shape;

            auto id = m_world->createBody(bodyInfo, true);
        }
    }

    for(int i = 0; i < 1; ++i)
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(-17.0f + i * 0.1f, i * 2.0f - 0.5f);
        bodyInfo.m_mass = 1.0f;
        bodyInfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 1.5f);

        auto id = m_world->createBody(bodyInfo, true);
        //m_world->setBodyLinearVelocity(id, cvVec2f(90.5f, 0));
        //m_world->setBodyAngularVelocity(m_Id, 0.8f);
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
