#include "CompoundShape.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCompoundShape.h>
#include <world/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

shared_ptr<cvShape> makeTumblur(float size)
{
    vector<cvCompoundShape::ShapeInstance> shapeInstances;
    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(size, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(size, 0);
        inst.m_transform.m_Rotation = DEG2RAD(90);

        shapeInstances.push_back(inst);
    }

    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(size, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(-size, 0);
        inst.m_transform.m_Rotation = DEG2RAD(90);

        shapeInstances.push_back(inst);
    }
    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(size, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(0, size);

        shapeInstances.push_back(inst);
    }
    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(size, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(0, -size);

        shapeInstances.push_back(inst);
    }
    auto compound = make_shared<cvCompoundShape>(shapeInstances);

    return compound;
}

CompoundShape::CompoundShape()
{    
    cvWorldCInfo cInfo;
    //cInfo.m_bpAABBExpesnion = 0.2f;
    //cInfo.m_gravity = cvVec2f::getZero();
    m_world = new cvWorld(cInfo);


    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Kinematic;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, 0);
        //bodyInfo.m_initTransform.m_Rotation = DEG2RAD(45);

        bodyInfo.m_shape = makeTumblur(10);;

        auto id = m_world->createBody(bodyInfo, true);
        m_world->setBodyAngularVelocity(id, DEG2RAD(30));
    }

    {
        // some boxes
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Dynamic;
        bodyInfo.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(0.25f, 0.25f), 0.02f));
        // some dynamic stuff
        for(int j = 0; j < 10; ++j)
        {
            for(int i = 0; i < 10; ++i)
            {
                bodyInfo.m_initTransform.m_Translation = cvVec2f(-4.5f + i * 0.5f, 0.5f * j );
                m_world->createBody(bodyInfo, true);
            }
        }
    }

    {
        // some boxes
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Dynamic;
        bodyInfo.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(0.25f, 0.5f), 0.02f));
        // some dynamic stuff
        for(int j = 0; j < 10; ++j)
        {
            for(int i = 0; i < 10; ++i)
            {
                bodyInfo.m_initTransform.m_Translation = cvVec2f(4.5f + i * 0.5f,  j );
                m_world->createBody(bodyInfo, true);
            }
        }
    }

    if(false)
    {
        // some circle
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Dynamic;
        bodyInfo.m_shape = make_shared<cvCircle>(cvVec2f(0, 0), 0.25f);
        // some dynamic stuff
        for(int j = 0; j < 10; ++j)
        {
            for(int i = 0; i < 10; ++i)
            {
                bodyInfo.m_initTransform.m_Translation = cvVec2f(4.5f + i * 0.5f, 0.5f * j );
                m_world->createBody(bodyInfo, true);
            }
        }
    }
}

void CompoundShape::tick(cvDebugDraw& debugDraw, float dt)
{
    cvSimInfo simInfo;
    simInfo.deltaTime = dt;
    m_world->simulate(simInfo);
    debugDraw.DrawWorld(*m_world);
}

REGISTER_TEST("CompoundShape", [](){return new CompoundShape();});
