#pragma once

#include "TestBase.h"
#include <world/cvBody.h>
#include <memory>

class cvConvexShape;
class Stacks: public TestBase
{
public:
    Stacks();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    cvWorld* m_world = nullptr;
    std::shared_ptr<cvConvexShape> m_shape;
    cvBodyId m_Id;
};
