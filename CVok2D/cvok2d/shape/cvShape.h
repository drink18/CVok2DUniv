#pragma once

#include <algorithm>
#include <core/cvAabb.h>

struct SuportInfo
{
    cvVec2f p;
    int index;

    SuportInfo(const cvVec2f& v, int i)
        :p(v), index(i){ }
};

#undef min
#undef max

class cvShape
{
public:
    enum ShapeType
    {
        eCircle,
        ePolygon,
        eCompoundShape,
        eShapeType_Count
    };

    virtual ShapeType getShapeType() const = 0;
    virtual ~cvShape() {}

    const cvAabb& getAabb() const {return m_aabb;}
    const void getAabb(const cvTransform& trans, cvAabb& outAabb) const
    {
        outAabb = m_aabb;
        outAabb.transform(trans);
    }

    virtual void updateAabb() = 0;
protected:
    cvAabb m_aabb;
};
