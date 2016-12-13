#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include <vector>

class cvConvexShape : public cvShape
{
public:
    virtual cvVec2f getSupport(const cvVec2f& direction) const = 0;
};
