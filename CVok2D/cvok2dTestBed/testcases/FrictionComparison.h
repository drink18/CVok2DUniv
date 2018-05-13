#pragma once

#include "TestBase.h"
#include <world/cvBody.h>
#include <memory>

class cvConvexShape;
class FrictionComparison: public TestBase
{
public:
    FrictionComparison();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    cvWorld* m_world = nullptr;
};
