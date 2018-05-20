#pragma once

#include "TestBase.h"
#include <world/cvBody.h>
#include <memory>

class cvConvexShape;
class WorldIntegration : public TestBase
{
public:
    WorldIntegration();


    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    std::shared_ptr<cvConvexShape> m_shape;
    cvBodyId m_Id;
};
