#pragma once

#include "TestBase.h"
#include <simulation/cvBody.h>
#include <memory>

class cvConvexShape;
class WorldIntegration : public TestBase
{
public:
    WorldIntegration();


    virtual void tick(cvDebugDraw& gdbDraw) override;

private:
    cvWorld* m_world = nullptr;
    std::shared_ptr<cvConvexShape> m_shape;
    cvBodyId m_Id;
};
