#include "Stacks.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCompoundShape.h>
#include <world/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

Stacks::Stacks()
{
    m_shape = shared_ptr<cvPolygonShape> (
            cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));

    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);
        vector<cvCompoundShape::ShapeInstance> shapeInsts;

        {
            cvCompoundShape::ShapeInstance inst0;
            inst0.m_shape = shared_ptr<cvPolygonShape> (
                    cvPolygonShape::createBox(cvVec2f(20.0f, 0.5f), 0.05f));
            inst0.m_transform.m_Translation = cvVec2f(5.0f, -15.0f);
            shapeInsts.push_back(inst0);
        }
        {
            cvCompoundShape::ShapeInstance inst0;
            inst0.m_shape = shared_ptr<cvPolygonShape> (
                    cvPolygonShape::createBox(cvVec2f(20.0f, 0.5f), 0.05f));
            inst0.m_transform.m_Translation = cvVec2f(-15.0f, -15.0f);
            inst0.m_transform.m_Rotation = -45;
            shapeInsts.push_back(inst0);
        }
        {
            cvCompoundShape::ShapeInstance inst0;
            inst0.m_shape = shared_ptr<cvPolygonShape> (
                    cvPolygonShape::createBox(cvVec2f(20.0f, 0.5f), 0.05f));
            inst0.m_transform.m_Translation = cvVec2f(25.0f, -15.0f);
            inst0.m_transform.m_Rotation = 45;
            shapeInsts.push_back(inst0);
        }


        bodyInfo.m_shape = make_shared<cvCompoundShape>(shapeInsts);

        auto id = m_world->createBody(bodyInfo, true);
    }

    // boxes
    for(int i = 0; i < 8; ++i)
    {
        for(int j = 0; j < 5; ++j)
        {
            cvBodyCInfo bodyInfo;
            bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f + 2.1f * j, 2.1f * i - 20.5f);
            bodyInfo.m_mass = 1.0f;
            bodyInfo.m_shape = m_shape;

            auto id = m_world->createBody(bodyInfo, true);
        }
    }

    for(int i = 0; i < 0; ++i)
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
