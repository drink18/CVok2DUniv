#include "BasicTest.h"
#include <DebugDraw.h>

BasicTest::BasicTest()
{
}

void BasicTest::tick(cvDebugDraw& dbgDraw)
{
    dbgDraw.AddPoint(cvVec2f(0, 0), 2.0f, cvColorf(1, 1, 1));
    dbgDraw.AddLine(cvVec2f(0, 0), cvVec2f(23, 23), cvColorf(1, 1, 0));
}
