#pragma once

#include <algorithm>
#include <core/cvAabb.h>

struct SimplexVertex
{
    cvVec2f p;
    int index;
    float u;

    SimplexVertex(const cvVec2f& v, int i, float _u)
        :p(v), index(i), u(_u) { }
};

class cvShape
{
public:
    enum ShapeType
    {
        eCircle,
        ePolygon,
        eShapeType_Count
    };
    virtual ShapeType getShapeType() const = 0;
    virtual ~cvShape() {}

    const cvAabb& getAabb() const {return m_aabb;}
    const void getAabb(const cvTransform& trans, cvAabb& outAabb) const
    {
        cvVec2f vert[4];
        vert[0] = m_aabb.m_Min;
        vert[1].set(m_aabb.m_Max.m_x, m_aabb.m_Min.m_y);
        vert[2].set(m_aabb.m_Max.m_x, m_aabb.m_Max.m_y);
        vert[4].set(m_aabb.m_Min.m_x, m_aabb.m_Max.m_y);

        cvMat33 m;
        trans.toMat33(m);
        for(auto & v : vert)
            v = m * v;

        cvVec2f min, max;
        min = vert[0];
        max = vert[0];

        for(int i = 0; i < 4; ++i)
        {
            min.m_x = std::min(min.m_x, vert[i].m_x);
            min.m_y = std::min(min.m_y, vert[i].m_y);
            max.m_x = std::max(max.m_x, vert[i].m_x);
            max.m_y = std::max(max.m_y, vert[i].m_y);
        }

        outAabb.m_Min = min;
        outAabb.m_Max = max;
    }

    virtual void updateAabb() = 0;
protected:
    cvAabb m_aabb;
};
