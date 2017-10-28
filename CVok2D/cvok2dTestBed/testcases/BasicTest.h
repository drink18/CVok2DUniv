#pragma once
#include "TestBase.h"

class BasicTest : public TestBase
{
public:
    BasicTest();

    virtual void tick(cvDebugDraw& gdbDraw) override;
};
