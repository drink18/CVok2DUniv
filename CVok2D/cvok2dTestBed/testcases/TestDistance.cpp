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
    m_circle = make_shared<cvCircle>(cvVec2f(0, 0), 1.0f);
}

void ClosestPointTest::updateShapePair(cvDebugDraw& gdbDraw, const cvCircle& circle, const cvPolygonShape& poly, 
        const cvTransform& t1, const cvTransform& t2)
{
        gdbDraw.DrawShape(circle, t1, cvColorf::Red);
        gdbDraw.DrawShape(poly, t2, cvColorf::Blue);

        cvMat33 m1; t1.toMat33(m1);
        cvMat33 m2; t2.toMat33(m2);

        auto fn = cvGetCollisionFn(circle.getShapeType(), poly.getShapeType());
        cvManifold manifold;
        (*fn)(circle, poly, m2, m1, manifold);
        cvManifoldPoint mp = manifold.m_points[0];
        auto pa = mp.m_point;
        auto pb = pa + mp.m_normal * mp.m_distance;

        gdbDraw.AddLine(pa, pb, cvColorf::Green);
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw, float dt)
{
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

}

REGISTER_TEST("TestDistance", [](){return new ClosestPointTest();});
