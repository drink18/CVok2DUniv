#include "cvPolygonShape.h"


cvVec2f cvPolygonShape::getSupport(const cvVec2f& direction) const
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
