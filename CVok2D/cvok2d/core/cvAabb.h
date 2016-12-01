#pragma once

#include "cvMath.h"

class cvAabb
{
public:
	cvAabb(const cvVec2f& min, const cvVec2f& max) :m_Max(max), m_Min(min) {}
	cvAabb() { setEmpty(); }
	inline void setEmpty();
	inline void include(const cvAabb& other);
	inline bool contains(const cvAabb& other) const;
	inline bool overlaps(const cvAabb& other) const;
public:
	cvVec2f m_Min;
	cvVec2f m_Max;
};

#include "cvAabb.inl"