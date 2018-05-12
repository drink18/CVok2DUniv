#include "CompoundShape.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCompoundShape.h>
#include <world/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

CompoundShape::CompoundShape()
{    
    cvWorldCInfo cInfo;
    //cInfo.m_bpAABBExpesnion = 0.2f;
    //cInfo.m_gravity = cvVec2f::getZero();
    m_world = new cvWorld(cInfo);

    vector<cvCompoundShape::ShapeInstance> shapeInstances;
    
    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(5.0f, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(5.0f, 0);
        inst.m_transform.m_Rotation = DEG2RAD(90);

        shapeInstances.push_back(inst);
    }

    {
        cvCompoundShape::ShapeInstance inst;
        inst.m_shape = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(5.0f, 0.2f), 0.01f));
        inst.m_transform.m_Translation = cvVec2f(-5.0f, 0);
        inst.m_transform.m_Rotation = DEG2RAD(90);

        shapeInstances.push_back(inst);
    }
    auto compound = make_shared<cvCompoundShape>(shapeInstances);

    // adding a static plane
    {
        cvBodyCInfo bodyInfo;
        bodyInfo.m_motionType = cvMotion::MotionType::Static;
        bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, -15.0f);
        bodyInfo.m_initTransform.m_Rotation = DEG2RAD(45);

        bodyInfo.m_shape = compound;

        auto id = m_world->createBody(bodyInfo, true);
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
