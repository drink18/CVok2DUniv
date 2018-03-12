#include "BasicTest.h"
#include <DebugDraw.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCircle.h>
#include <simulation/cvWorld.h>

BasicTest::BasicTest()
{
    m_box = cvPolygonShape::createBox(cvVec2f(5, 5), 0.01f);
    m_circle = new cvCircle(cvVec2f(0, 0), 3);

    cvWorldCInfo cinfo;
    m_world = new cvWorld(cinfo);

    cvBodyCInfo bInfo;
    bInfo.m_shape.reset(m_box);
    bInfo.m_initTransform.setIdentity();
    bInfo.m_initTransform.m_Translation = cvVec2f(0, 0);
    //bInfo.m_initTransform.m_Rotation = DEG2RAD(20);
    m_world->createBody(bInfo, true);

    bInfo.m_shape.reset(m_circle);
    bInfo.m_initTransform.m_Rotation = 0;
    m_world->createBody(bInfo, true);
}

void BasicTest::tick(cvDebugDraw& dbgDraw, float dt)
{
    dbgDraw.AddPoint(cvVec2f(0, 0), 3.0f, cvColorf::White);
    //cvTransform trans;
    //trans.m_Rotation = DEG2RAD(10);
    //dbgDraw.DrawShape(*m_box, trans);
    //dbgDraw.DrawShape(*m_circle, trans);



    dbgDraw.DrawWorld(*m_world);
}

TestBase* s_CreateBasicTest()
{
    return new BasicTest();
}

REGISTER_TEST("BasicTest", [](){return s_CreateBasicTest();});
