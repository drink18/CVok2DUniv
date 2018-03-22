#include "TestCollisionDispatch.h"
#include <cvok2dInit.h>
#include <world/cvWorld.h>
#include <world/cvBroadPhaseSAP.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include "../DebugDraw.h"

TestCollisionDispatch::TestCollisionDispatch()
{
    cv2DInit();
    cvWorldCInfo info;
	info.m_bpAABBExpesnion = 0.5f;
    m_world.reset(new cvWorld(info));

    {
        cvCircle* circle = new cvCircle(cvVec2f(0.0f, 0.0f), 1.0f);
        cvBodyCInfo binfo;
        binfo.m_initTransform.m_Translation.set(1.05f, 0.0f);
        binfo.m_shape.reset(circle);
        m_world->createBody(binfo, true);
    }

    {
        cvConvexShape* box = cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.01f);
        cvBodyCInfo binfo;
        binfo.m_initTransform.m_Translation.set(-1.05f, 0.0f);
        binfo.m_shape.reset(box);
        m_world->createBody(binfo, true);
    }


}

void TestCollisionDispatch::tick(cvDebugDraw& dbgDraw, float dt)
{
    cvSimInfo si;
    si.deltaTime = dt;
    m_world->simulate(si);

    dbgDraw.DrawWorld(*m_world);
}

REGISTER_TEST("TestCollisionDispatch", [](){return new TestCollisionDispatch();});
