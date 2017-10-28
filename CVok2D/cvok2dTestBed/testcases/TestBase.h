#include <cvok2d.h>
#pragma once

class cvDebugDraw;
class cvWorld;

class TestBase
{
public:
    TestBase ();

    virtual void tick(cvDebugDraw& dbgDraw);
};
