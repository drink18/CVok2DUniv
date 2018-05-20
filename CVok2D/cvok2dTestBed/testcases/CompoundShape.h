#pragma once

#include "TestBase.h"
#include <world/cvBody.h>
#include <memory>

class cvConvexShape;
class CompoundShape: public TestBase
{
public:
    CompoundShape();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    std::shared_ptr<CompoundShape> m_shape;
};
