#include "cvCore.h"

#include "cvMathDefs.h"
#include "cvMath.h"

float cvVec2f::length() const { return cvSqrt(sqrLength()); }
float cvVec2f::sqrLength() const { return m_x * m_x + m_y * m_y; }

void cvVec2f::add(const cvVec2f& v)
{
	setAdd(*this, v);
}

void cvVec2f::setNegate()
{
	setScale(-1);
}

void cvVec2f::sub(const cvVec2f& v)
{
	setSub(*this, v);
}

void cvVec2f::setAdd(const cvVec2f& v1, const cvVec2f& v2)
{
	m_x = v1.m_x + v2.m_x;
	m_y = v1.m_y + v2.m_y;
}

void cvVec2f::addMul(const cvVec2f& v, float s)
{
    m_x += v.m_x * s;
    m_y += v.m_y * s;
}

void cvVec2f::setSub(const cvVec2f& v1, const cvVec2f& v2)
{
	m_x = v1.m_x - v2.m_x;
	m_y = v1.m_y - v2.m_y;
}

void cvVec2f::setScale(float s)
{
	m_x *= s;
	m_y *= s;
}

float cvVec2f::dot(const cvVec2f& v1) const
{
	return m_x * v1.m_x + m_y * v1.m_y;
}

void cvVec2f::normalize()
{
	float len = length();
	cvAssert(len > CV_FLOAT_EPS);
	m_x /= len;
	m_y /= len;
}

cvVec2f cvVec2f::getNormalized() const
{
	cvVec2f cv(m_x, m_y);
	cv.normalize();
	return cv;
}

/// project this to v
void cvVec2f::setProj(const cvVec2f& v)
{
	cvAssert(v.length() > CV_FLOAT_EPS);

	cvVec2f v1(v);
	v1.normalize();
	float d = dot(v1);
	m_x = v1.m_x * d;
	m_y = v1.m_y * d;
}

cvVec2f cvVec2f::project(const cvVec2f& v) const
{
	cvVec2f pv(*this);
	pv.setProj(v);
	return pv;
}

// this cross v
float cvVec2f::cross(const cvVec2f& v) const
{
	return m_x * v.m_y - m_y * v.m_x;
}

float cvVec2f::distance(const cvVec2f& v) const
{
	cvVec2f v1; v1.setSub(*this, v);
	return v1.length();
}

cvVec2f	cvVec2f::min2(const cvVec2f& v1, const cvVec2f& v2)
{
	cvVec2f v;
	v.m_x = std::fmin(v1.m_x, v2.m_x);
	v.m_y = std::fmin(v1.m_y, v2.m_y);
	return v;
}

cvVec2f cvVec2f::max2(const cvVec2f& v1, const cvVec2f& v2)
{
	cvVec2f v;
	v.m_x = std::fmax(v1.m_x, v2.m_x);
	v.m_y = std::fmax(v1.m_y, v2.m_y);
	return v;
}

bool cvVec2f::equal(const cvVec2f& v1, const cvVec2f& v2)
{
	return almost_equal(v1.m_x, v2.m_x) && almost_equal(v1.m_y, v2.m_y);
}

bool cvVec2f::less(const cvVec2f& v1, const cvVec2f& v2)
{
	return v1.m_x < v2.m_x && v1.m_y < v2.m_y;
}

bool cvVec2f::lessOrEqual(const cvVec2f& v1, const cvVec2f& v2)
{
	return cvVec2f::equal(v1, v2) || cvVec2f::less(v1, v2);
}

bool cvVec2f::greater(const cvVec2f& v1, const cvVec2f& v2)
{
	return v1.m_x > v2.m_x && v1.m_y > v2.m_y;
}

bool cvVec2f::greatOrEqual(const cvVec2f& v1, const cvVec2f& v2)
{
	return cvVec2f::equal(v1, v2) || cvVec2f::greater(v1, v2);
}
