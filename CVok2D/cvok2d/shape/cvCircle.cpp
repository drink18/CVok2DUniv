#include "cvCircle.h"

void cvCircle::updateAabb()
{
    cvVec2f radiusV(m_radius, m_radius);
    m_aabb.m_Min = m_center - radiusV;
    m_aabb.m_Max = m_center + radiusV;
}

cvVec2f cvCircle::getSupport(const cvVec2f& direction) const
{
    cvVec2f support = m_center;
    support.addMul(direction, m_radius);
    return support;
}
