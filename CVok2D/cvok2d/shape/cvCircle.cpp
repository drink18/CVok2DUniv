#include "cvCircle.h"

void cvCircle::updateAabb()
{
    cvVec2f radiusV(m_radius, m_radius);
    m_aabb.m_Min = m_center - radiusV;
    m_aabb.m_Max = m_center + radiusV;
}
