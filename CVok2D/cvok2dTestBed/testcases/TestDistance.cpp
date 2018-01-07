#include "TestDistance.h"
#include <collision/cvDistance.h>
#include <DebugDraw.h>
#include <cstdlib>

ClosestPointTest::ClosestPointTest()
{
}

void ClosestPointTest::tick(cvDebugDraw& gdbDraw)
{
    cvVec2f a(0, 10);
    cvVec2f b(-10, 0);
    cvVec2f c(10, 0);

    gdbDraw.AddLine(a, b, cvColorf(1.0f, 1.0f, 1.0f));
    gdbDraw.AddLine(c, b, cvColorf(1.0f, 1.0f, 1.0f));
    gdbDraw.AddLine(a, c, cvColorf(1.0f, 1.0f, 1.0f));

    std::srand(0);
    for(int i = 0; i < 10; ++i)
    {
        float x = 50 * (0.5f - (float)std::rand() / RAND_MAX);
        float y = 50 * (0.5f - (float)std::rand() / RAND_MAX);

        cvVec2f q(x, y);
        auto res = cvDist::pointDistanceToTriangle(q, a, b, c);
        gdbDraw.AddLine(q, res.pt, cvColorf(1.0f, 0.0f, 1.0f));
    }

}
