#pragma once

#include "TestBase.h"


class WorldIntegration : public TestBase
{
public:
    WorldIntegration();


    virtual void tick(cvDebugDraw& gdbDraw) override;

private:
    cvWorld* m_world = nullptr;

};
