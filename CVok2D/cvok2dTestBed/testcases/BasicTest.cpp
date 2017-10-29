#include "BasicTest.h"
#include <DebugDraw.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCircle.h>

BasicTest::BasicTest()
{
    m_box = cvPolygonShape::createBox(cvVec2f(5, 5), 0.01f);
    m_circle = new cvCircle(cvVec2f(10, 0), 3);
}

void BasicTest::tick(cvDebugDraw& dbgDraw)
{
    dbgDraw.AddPoint(cvVec2f(0, 0), 2.0f, cvColorf(1, 1, 1));
    dbgDraw.AddLine(cvVec2f(0, 0), cvVec2f(23, 23), cvColorf(1, 1, 0));

    dbgDraw.DrawShape(*m_box);
    dbgDraw.DrawShape(*m_circle);
}
