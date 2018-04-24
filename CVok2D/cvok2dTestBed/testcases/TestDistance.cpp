#include "TestDistance.h"
#include <collision/cvDistance.h>
#include <collision/cvCollisionDispatch.h>
#include <collision/GJK.h>
#include <collision/SAT.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCircle.h>
#include <DebugDraw.h>
#include <collision/cvManifold.h>
#include <cstdlib>

using namespace GJK;
using namespace SAT;

ClosestPointTest::ClosestPointTest()
{
    m_poly = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(1.0f, 1.0f), 0.05f));
    m_poly1 = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(0.8f, 0.8f), 0.05f));
    m_circle = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
}

void ClosestPointTest::updateShapePair(cvDebugDraw& gdbDraw, const cvConvexShape& poly, 
        const cvConvexShape& poly1, const cvTransform& t1, const cvTransform& t2)
{
        gdbDraw.DrawShape(poly, t1, cvColorf::Blue);
        gdbDraw.DrawShape(poly1, t2, cvColorf::Red);

        cvMat33 m1; t1.toMat33(m1);
        cvMat33 m2; t2.toMat33(m2);

        auto fn = cvGetCollisionFn(poly.getShapeType(), poly1.getShapeType());
        cvManifold manifold;
        (*fn)(poly, poly1, m1, m2, manifold);

        for(int i = 0; i < manifold.m_numPt; ++i)
        {
            cvManifoldPoint mp = manifold.m_points[i];
            auto pa = mp.m_point;
            auto pb = pa + mp.m_normal * mp.m_distance;

            gdbDraw.AddLine(pa, pb, cvColorf::Green);
        }
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw, float dt)
{
    {
        cvTransform t1; t1.m_Translation.set(0, -5.0f);
        cvTransform t2; t2.m_Translation.set(2.5f, -5.0f);
        updateShapePair(gdbDraw, *m_circle, *m_circle, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(5, -5.0f);
        cvTransform t2; t2.m_Translation.set(6.8f, -5.0f);
        updateShapePair(gdbDraw, *m_circle, *m_circle, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(-5.0f, 0);
        cvTransform t2; t2.m_Translation.set(-2.5f, 0); t2.m_Rotation = DEG2RAD(45);
        updateShapePair(gdbDraw, *m_circle, *m_poly, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(0, 0);
        cvTransform t2; t2.m_Translation.set(2.5f, 0);
        updateShapePair(gdbDraw, *m_circle, *m_poly, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(5.0f, 0);
        cvTransform t2; t2.m_Translation.set(6.9f, 0);
        updateShapePair(gdbDraw, *m_circle, *m_poly, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(10.0f, 0);
        cvTransform t2; t2.m_Translation.set(10.9f, 0);
        updateShapePair(gdbDraw, *m_circle, *m_poly, t1, t2);
    }

    static float a = DEG2RAD(45);
    //a += dt * DEG2RAD(45);
    // box box 
    {
        cvTransform t1; t1.m_Translation.set(5.0f, 5.0f); t1.m_Rotation = a;
        cvTransform t2; t2.m_Translation.set(7.5f, 5.0f);
        updateShapePair(gdbDraw, *m_poly, *m_poly1, t1, t2);
    }

    {
        cvTransform t1; t1.m_Translation.set(15.0f, 5.0f);
        cvTransform t2; t2.m_Translation.set(16.0f, 5.0f);
        updateShapePair(gdbDraw, *m_poly, *m_poly1, t1, t2);
    }

}

REGISTER_TEST("TestDistance", [](){return new ClosestPointTest();});
