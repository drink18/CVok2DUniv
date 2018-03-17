#pragma once

#include "cvMath.h"

class cvAabb
{
public:
	cvAabb(const cvVec2f& min, const cvVec2f& max) : m_Min(min), m_Max(max) {}
	cvAabb() { setEmpty(); }
	inline void setEmpty();
	inline void include(const cvAabb& other);
	inline bool contains(const cvAabb& other) const;
	inline bool overlaps(const cvAabb& other) const;
    inline void expand(const cvVec2f& expansion);
public:
	cvVec2f m_Min;
	cvVec2f m_Max;
};

#include "cvAabb.inl"
