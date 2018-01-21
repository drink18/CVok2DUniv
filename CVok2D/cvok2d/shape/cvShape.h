#pragma once

#include <algorithm>
#include <core/cvAabb.h>

struct SimplexVertex
{
    cvVec2f p;
    int index;
    float u;

    //extended info for shape vs shape
    cvVec2f sA; //support point on A
    cvVec2f sB; //support point on B

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
        vert[1].set(m_aabb.m_Max.x, m_aabb.m_Min.y);
        vert[2].set(m_aabb.m_Max.x, m_aabb.m_Max.y);
        vert[3].set(m_aabb.m_Min.x, m_aabb.m_Max.y);

        cvMat33 m;
        trans.toMat33(m);
        for(auto & v : vert)
            v = m * v;

        cvVec2f min, max;
        min = vert[0];
        max = vert[0];

        for(int i = 0; i < 4; ++i)
        {
            min.x = std::min(min.x, vert[i].x);
            min.y = std::min(min.y, vert[i].y);
            max.x = std::max(max.x, vert[i].x);
            max.y = std::max(max.y, vert[i].y);
        }

        outAabb.m_Min = min;
        outAabb.m_Max = max;
    }

    virtual void updateAabb() = 0;
protected:
    cvAabb m_aabb;
};
