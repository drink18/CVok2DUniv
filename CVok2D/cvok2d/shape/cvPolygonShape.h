#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include "cvConvexShape.h"

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

    virtual cvVec2f getSupport(const cvVec2f& direction) const override
    {
        float bestDot = direction.dot(m_vertices[0]);
        cvVec2f bestVert = m_vertices[0];

        for(int i = 1; i < m_vertices.size(); ++i)
        {
            float dot = direction.dot(m_vertices[i]);
            if(dot > bestDot)
            {
                bestDot = dot;
                bestVert = m_vertices[i];
            }
        }
        
        return bestVert;
    };
    
    std::vector<cvVec2f> m_vertices;
    float m_radius;
};
