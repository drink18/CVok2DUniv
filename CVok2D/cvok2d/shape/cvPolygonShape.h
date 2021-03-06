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

        updateAabb();
    }

    virtual ShapeType getShapeType() const override
    {
        return ePolygon;
    }

    virtual SuportInfo getSupport(const cvVec2f& direction) const override;

    static cvPolygonShape* createBox(const cvVec2f& halfExt, float radius);
    static cvPolygonShape* createBox(const cvVec2f& min, const cvVec2f& max, float radius);


    virtual void updateAabb() override;
private:
};
