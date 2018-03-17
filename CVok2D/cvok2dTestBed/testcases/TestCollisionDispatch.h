#pragma once
#include "TestBase.h"
#include <memory>

class TestCollisionDispatch : public TestBase
{
public:
    TestCollisionDispatch();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;

private:
    std::unique_ptr<cvWorld> m_world = nullptr;
};
