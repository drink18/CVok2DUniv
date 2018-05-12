#include "cvAabb.h"

void cvAabb::transform(const cvTransform& trans)
{
    cvMat33 m;
    trans.toMat33(m);
    transform(m);
}

void cvAabb::transform(const cvMat33& m)
{
        cvVec2f vert[4];
        vert[0] = m_Min;
        vert[1].set(m_Max.x, m_Min.y);
        vert[2].set(m_Max.x, m_Max.y);
        vert[3].set(m_Min.x, m_Max.y);

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

        m_Min = min;
        m_Max = max;
}
