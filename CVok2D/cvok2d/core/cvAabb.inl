#include "cvAabb.h"
#include <cfloat>

void cvAabb::setEmpty()
{
	m_Min.set(FLT_MAX, FLT_MAX);
	m_Max.set(FLT_MIN, FLT_MIN);
}

void cvAabb::include(const cvAabb& other)
{
	m_Min = cvVec2f::min2(other.m_Min, m_Min);
	m_Max = cvVec2f::max2(other.m_Max, m_Max);
}

bool cvAabb::contains(const cvAabb& other) const
{
	return cvVec2f::lessOrEqual(other.m_Max, m_Max) && cvVec2f::lessOrEqual(m_Min, other.m_Min);
}

bool cvAabb::overlaps(const cvAabb& other) const
{
	return cvVec2f::greater(m_Max, other.m_Min)
		&& cvVec2f::greater(other.m_Max, m_Min);
}

