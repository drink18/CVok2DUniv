#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include "cvConvexShape.h"

class cvCircle : public cvConvexShape
{
public:
    cvCircle(const cvVec2f& center, float radius)
    {
        m_vertices.push_back(center);
        m_radius = radius;
    }

    virtual ShapeType getShapeType() const override
    {
        return cvShape::eCircle;
    }

    virtual SuportInfo getSupport(const cvVec2f& direction) const override;

    virtual void updateAabb() override;

    float getRadius() const {return m_radius;}
    cvVec2f getCenter() const {return m_vertices[0];};
};
