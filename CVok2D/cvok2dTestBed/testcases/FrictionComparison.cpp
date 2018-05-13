#include "FrictionComparison.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCompoundShape.h>
#include <world/cvWorld.h>
#include <world/cvMaterial.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

void MakeTestSlopeAndObj(float y, cvWorld* world, float fric)
{
    auto platMat = make_shared<cvMaterial>();
    auto bodyMat = make_shared<cvMaterial>();
    platMat->m_friction = 0.0f;
    bodyMat->m_friction = fric;

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
        world->createBody(cinfo, true);


        cinfo.m_shape = shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.1f));
        cinfo.m_motionType = cvMotion::MotionType::Dynamic;
        cinfo.m_initTransform.m_Translation = cvVec2f(-12.0f, 7 + y);
        cinfo.m_material = bodyMat;
        world->createBody(cinfo, true);
    }
}

FrictionComparison::FrictionComparison()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    MakeTestSlopeAndObj(-20.0f, m_world, 1.0f);
    MakeTestSlopeAndObj(-10.0f, m_world, 0.5f);
    MakeTestSlopeAndObj(0.0f, m_world, 0.2f);
}

void FrictionComparison::tick(cvDebugDraw& debugDraw, float dt)
{
    cvSimInfo simInfo;
    simInfo.deltaTime = dt;
    m_world->simulate(simInfo);
    debugDraw.DrawWorld(*m_world);
}

REGISTER_TEST("FrictionComparison", [](){return new FrictionComparison();});
