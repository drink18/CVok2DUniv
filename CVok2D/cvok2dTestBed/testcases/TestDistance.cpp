#include "TestDistance.h"
#include <collision/cvDistance.h>
#include <collision/GJK.h>
#include <collision/SAT.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCircle.h>
#include <DebugDraw.h>
#include <cstdlib>

using namespace GJK;
using namespace SAT;

ClosestPointTest::ClosestPointTest()
{
    m_box = cvPolygonShape::createBox(cvVec2f(5, 5), cvVec2f(15, 15), 0.05f);
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw, float dt)
{
    if(0)
    {
        m_b1 = cvPolygonShape::createBox(cvVec2f(5, 5), 0.05f);
        auto b2 = cvPolygonShape::createBox(cvVec2f(7.5, 7.5), 0.05f);
        cvTransform& t1 = m_t1;
        t1.m_Translation.set(0, 0);
        t1.m_Rotation += DEG2RAD(45) * dt;

        cvTransform &t2 = m_t2;
        t2.m_Translation.set(19, 0);//17, 17);
        t2.m_Rotation -= DEG2RAD(45) * dt;

        gdbDraw.DrawShape(*m_b1, t1, cvColorf::Red);
        gdbDraw.DrawShape(*b2, t2, cvColorf::Blue);

        cvMat33 m1; t1.toMat33(m1);
        cvMat33 m2; t2.toMat33(m2);

        cvShapeQueryInput input(*m_b1, *b2, m1, m2);
        auto res = cvGJKConvexToConvex(input);

        gdbDraw.AddLine(res.m_pA, res.m_pB, cvColorf::Green);
    }

    {
        m_b1 = cvPolygonShape::createBox(cvVec2f(5, 5), 0.05f);
        auto b2 = new cvCircle(cvVec2f(0,0), 5.0f);
        cvTransform& t1 = m_t1;
        t1.m_Translation.set(0, 0);
        t1.m_Rotation += DEG2RAD(50) * dt;

        cvTransform &t2 = m_t2;
        t2.m_Translation.set(10, 0);//17, 17);
        t2.m_Rotation -= DEG2RAD(45) * dt;

        gdbDraw.DrawShape(*m_b1, t1, cvColorf::Red);
        gdbDraw.DrawShape(*b2, t2, cvColorf::Blue);

        cvMat33 m1; t1.toMat33(m1);
        cvMat33 m2; t2.toMat33(m2);

        cvShapeQueryInput input(*m_b1, *b2, m1, m2);
        auto res =_circleToPolygon(*b2, *m_b1, m2, m1);

        auto pa = res.closetPt;
        auto pb = pa + res.normal * res.distance;
        if(res.penetrated)
        {
            gdbDraw.AddLine(pa, pb, cvColorf::Green);
            gdbDraw.AddLine(res.ep0, res.ep1, cvColorf::Green);
        }
    }
}

REGISTER_TEST("TestDistance", [](){return new ClosestPointTest();});
