#pragma once
#include "TestBase.h"

class cvPolygonShape;
class cvCircle;

class BasicTest : public TestBase
{
public:
    BasicTest();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    cvPolygonShape* m_box = nullptr;
    cvCircle* m_circle = nullptr;
    cvWorld* m_world = nullptr;
};

