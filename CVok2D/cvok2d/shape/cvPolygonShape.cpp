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
}

cvPolygonShape* cvPolygonShape::createBox(const cvVec2f& halfExt, float radius)
{

    return createBox(cvVec2f(-halfExt.m_x, -halfExt.m_y), cvVec2f(halfExt.m_x, halfExt.m_y), radius);
}

cvPolygonShape* cvPolygonShape::createBox(const cvVec2f& min, const cvVec2f& max, float radius)
{
    cvVec2f verts[4];
    verts[0].m_x = min.m_x;
    verts[0].m_y = min.m_y;
    verts[1].m_x = max.m_x;
    verts[1].m_y = min.m_y;
    verts[2].m_x = max.m_x;
    verts[2].m_y = max.m_y;
    verts[3].m_x = min.m_x;
    verts[3].m_y = max.m_y;

    return new cvPolygonShape(verts, 4, radius);
}

void cvPolygonShape::updateAabb()
{
    cvVec2f min = m_vertices[0];
    cvVec2f max = m_vertices[0];

    for(int i = 1; i < m_vertices.size(); ++i)
    {
        auto vert = m_vertices[i];
        min.m_x = std::min(min.m_x, vert.m_x);
        min.m_y = std::min(min.m_y, vert.m_y);
        max.m_x = std::max(max.m_x, vert.m_x);
        max.m_y = std::max(max.m_y, vert.m_y);
    }
}
