#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include <vector>

class cvConvexShape : public cvShape
{
public:
    virtual SuportInfo getSupport(const cvVec2f& direction) const = 0;
    const std::vector<cvVec2f>& getVertices() const {return m_vertices;};

    float getRadius() const {return m_radius;}
protected:
    float m_radius;
    std::vector<cvVec2f> m_vertices;
};
