#include "cvPolygonShape.h"


SuportInfo cvPolygonShape::getSupport(const cvVec2f& direction) const
{
    float bestDot = direction.dot(m_vertices[0]);
    cvVec2f bestVert = m_vertices[0];

    int i = 0;
    int bestIdx = 0;
    for(i = 1; i < m_vertices.size(); ++i)
    {
        float dot = direction.dot(m_vertices[i]);
        if(dot > bestDot)
        {
            bestDot = dot;
            bestIdx = i;
            bestVert = m_vertices[i];
        }
    }

    SuportInfo sv(bestVert, bestIdx);

    return sv;
}

cvPolygonShape* cvPolygonShape::createBox(const cvVec2f& halfExt, float radius)
{

    return createBox(cvVec2f(-halfExt.x, -halfExt.y), cvVec2f(halfExt.x, halfExt.y), radius);
}

cvPolygonShape* cvPolygonShape::createBox(const cvVec2f& min, const cvVec2f& max, float radius)
{
    cvVec2f verts[4];
    verts[0].x = min.x;
    verts[0].y = min.y;
    verts[1].x = max.x;
    verts[1].y = min.y;
    verts[2].x = max.x;
    verts[2].y = max.y;
    verts[3].x = min.x;
    verts[3].y = max.y;

    return new cvPolygonShape(verts, 4, radius);
}

void cvPolygonShape::updateAabb()
{
    cvVec2f min = m_vertices[0];
    cvVec2f max = m_vertices[0];

    for(int i = 1; i < m_vertices.size(); ++i)
    {
        auto vert = m_vertices[i];
        min.x = std::min(min.x, vert.x);
        min.y = std::min(min.y, vert.y);
        max.x = std::max(max.x, vert.x);
        max.y = std::max(max.y, vert.y);
    }
}
