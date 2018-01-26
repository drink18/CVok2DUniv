#pragma once
#include "TestBase.h"

class cvPolygonShape;
class cvCircle;

class ClosestPointTest: public TestBase
{
public:
    ClosestPointTest();

    virtual void tick(cvDebugDraw& gdbDraw) override;

private:
    cvPolygonShape* m_box;
    cvPolygonShape* m_b1;
    cvTransform m_t1;
};
