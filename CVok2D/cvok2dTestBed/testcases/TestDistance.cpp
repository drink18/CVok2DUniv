#include "TestDistance.h"
#include <collision/cvDistance.h>
#include <collision/GJK.h>
#include <shape/cvPolygonShape.h>
#include <DebugDraw.h>
#include <cstdlib>

ClosestPointTest::ClosestPointTest()
{
    m_box = cvPolygonShape::createBox(cvVec2f(5, 5), cvVec2f(15, 15), 0.05f);
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw)
{
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
}
