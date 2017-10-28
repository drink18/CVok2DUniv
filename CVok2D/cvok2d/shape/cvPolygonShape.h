#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include "cvConvexShape.h"
#include <vector>

using namespace std;
class cvPolygonShape : public cvConvexShape
{
public:
    cvPolygonShape(const cvVec2f* vertices, int numVertices, float radius)
    {
        m_vertices.reserve(numVertices);
        m_vertices.insert(m_vertices.end(), vertices, vertices + numVertices);
        m_radius = radius;;
    }

    virtual ShapeType getShapeType() const override
    {
        return ePolygon;
    }

    virtual cvVec2f getSupport(const cvVec2f& direction) const override;

    vector<cvVec2f> m_vertices;
    float m_radius;
};
