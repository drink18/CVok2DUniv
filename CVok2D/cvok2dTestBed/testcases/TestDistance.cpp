#include "TestDistance.h"
#include <collision/cvDistance.h>
#include <collision/GJK.h>
#include <shape/cvPolygonShape.h>
#include <DebugDraw.h>
#include <cstdlib>

using namespace GJK;
ClosestPointTest::ClosestPointTest()
{
    m_box = cvPolygonShape::createBox(cvVec2f(5, 5), cvVec2f(15, 15), 0.05f);
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw)
{
#if 0
    {
        cvVec2f a(0, 10);
        cvVec2f b(-10, 0);
        cvVec2f c(10, 0);

        gdbDraw.AddLine(a, b, cvColorf::White);
        gdbDraw.AddLine(c, b, cvColorf::Yellow);
        gdbDraw.AddLine(a, c, cvColorf::Cyan);

        std::srand(0);
        for (int i = 0; i < 10; ++i)
        {
            float x = 50 * (0.5f - (float)std::rand() / RAND_MAX);
            float y = 50 * (0.5f - (float)std::rand() / RAND_MAX);

            cvVec2f q(x, y);
            auto res = cvDist::pointDistanceToTriangle(q, a, b, c);
            gdbDraw.AddLine(q, res.pt, cvColorf::Purple);
        }

    }

    {
        cvVec2f q(0, 0);
        auto gjkRes = GJK::pointToConvex(q, *m_box);

        gdbDraw.DrawShape(*m_box, cvTransform::getIdentity(), cvColorf::Cyan);
        gdbDraw.AddLine(q, gjkRes.closetPt, cvColorf::Yellow);
    }

    {
        cvVec2f q(0, 7.2f);
        auto gjkRes = GJK::pointToConvex(q, *m_box);

        gdbDraw.DrawShape(*m_box, cvTransform::getIdentity(), cvColorf::Cyan);
        gdbDraw.AddLine(q, gjkRes.closetPt, cvColorf::Yellow);
    }

    {
        cvVec2f q(18, 7.2f);
        auto gjkRes = GJK::pointToConvex(q, *m_box);

        gdbDraw.DrawShape(*m_box, cvTransform::getIdentity(), cvColorf::Cyan);
        gdbDraw.AddLine(q, gjkRes.closetPt, cvColorf::Yellow);
    }

    {
        cvVec2f q(18, 18);
        auto gjkRes = GJK::pointToConvex(q, *m_box);

        gdbDraw.DrawShape(*m_box, cvTransform::getIdentity(), cvColorf::Cyan);
        gdbDraw.AddLine(q, gjkRes.closetPt, cvColorf::Yellow);
    }
#endif

    {
        auto b1 = cvPolygonShape::createBox(cvVec2f(5, 5), 0.05f);
        auto b2 = cvPolygonShape::createBox(cvVec2f(7.5, 7.5), 0.05f);
        cvTransform t1;
        t1.m_Translation.set(0, 0);

        cvTransform t2;
        t2.m_Translation.set(17, 0);//17, 17);
        t2.m_Rotation = DEG2RAD(0);

        gdbDraw.DrawShape(*b1, t1, cvColorf::Red);
        gdbDraw.DrawShape(*b2, t2, cvColorf::Blue);

        cvMat33 m1; t1.toMat33(m1);
        cvMat33 m2; t2.toMat33(m2);

        cvShapeQueryInput input(*b1, *b2, m1, m2);
        auto res = cvGJKConvexToConvex(input);
        gdbDraw.AddLine(res.m_pA, res.m_pB, cvColorf::Green);
    }
}
