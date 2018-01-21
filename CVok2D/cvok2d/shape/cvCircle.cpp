#include "cvCircle.h"

void cvCircle::updateAabb()
{
    cvVec2f radiusV(getRadius(), getRadius());
    m_aabb.m_Min = getCenter() - radiusV;
    m_aabb.m_Max = getCenter() + radiusV;
}

SuportInfo cvCircle::getSupport(const cvVec2f& direction) const
{
    cvVec2f support = getCenter();
    support.addMul(direction, getRadius());
    SuportInfo v(support, 0);
    return v;
}
