#include "cvCompoundShape.h"

void cvCompoundShape::updateAabb()
{
    cvAabb a = m_shapeInstances[0].m_shape->getAabb();;
    a.transform(m_shapeInstances[0].m_transform);

    m_aabb = a;

    for(int i = 1; i < m_shapeInstances.size(); ++i)
    {
        a = m_shapeInstances[i].m_shape->getAabb();;
        a.transform(m_shapeInstances[i].m_transform);
        m_aabb.include(a);
    }
}
