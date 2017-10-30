#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include "cvConvexShape.h"

class cvCircle : public cvConvexShape
{
public:
    cvCircle(const cvVec2f& center, float radius)
        : m_center(center), m_radius(radius)
    {
    }

    cvVec2f m_center;
    float m_radius;
    virtual ShapeType getShapeType() const override
    {
        return cvShape::eCircle;
    }

    virtual cvVec2f getSupport(const cvVec2f& direction) const override
    {
        cvVec2f support = m_center;
        support.addMul(direction, m_radius);
        return support;
    }

    virtual void updateAabb() override;
};
