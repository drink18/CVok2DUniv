#include "cvCore.h"

#include "cvMathDefs.h"
#include "cvMath.h"

float cvVec2f::length() const { return cvSqrt(sqrLength()); }
float cvVec2f::sqrLength() const { return x * x + y * y; }

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
	x = v1.x + v2.x;
	y = v1.y + v2.y;
}

void cvVec2f::addMul(const cvVec2f& v, float s)
{
    x += v.x * s;
    y += v.y * s;
}

void cvVec2f::setSub(const cvVec2f& v1, const cvVec2f& v2)
{
	x = v1.x - v2.x;
	y = v1.y - v2.y;
}

void cvVec2f::setScale(float s)
{
	x *= s;
	y *= s;
}

float cvVec2f::dot(const cvVec2f& v1) const
{
	return x * v1.x + y * v1.y;
}

void cvVec2f::normalize()
{
	float len = length();
	cvAssert(len > CV_FLOAT_EPS);
	x /= len;
	y /= len;
}

cvVec2f cvVec2f::getNormalized() const
{
	cvVec2f cv(x, y);
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
	x = v1.x * d;
	y = v1.y * d;
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
	return x * v.y - y * v.x;
}

float cvVec2f::distance(const cvVec2f& v) const
{
	cvVec2f v1; v1.setSub(*this, v);
	return v1.length();
}

cvVec2f	cvVec2f::min2(const cvVec2f& v1, const cvVec2f& v2)
{
	cvVec2f v;
	v.x = std::fmin(v1.x, v2.x);
	v.y = std::fmin(v1.y, v2.y);
	return v;
}

cvVec2f cvVec2f::max2(const cvVec2f& v1, const cvVec2f& v2)
{
	cvVec2f v;
	v.x = std::fmax(v1.x, v2.x);
	v.y = std::fmax(v1.y, v2.y);
	return v;
}

bool cvVec2f::equal(const cvVec2f& v1, const cvVec2f& v2)
{
	return almost_equal(v1.x, v2.x) && almost_equal(v1.y, v2.y);
}

bool cvVec2f::less(const cvVec2f& v1, const cvVec2f& v2)
{
	return v1.x < v2.x && v1.y < v2.y;
}

bool cvVec2f::lessOrEqual(const cvVec2f& v1, const cvVec2f& v2)
{
	return cvVec2f::equal(v1, v2) || cvVec2f::less(v1, v2);
}

bool cvVec2f::greater(const cvVec2f& v1, const cvVec2f& v2)
{
	return v1.x > v2.x && v1.y > v2.y;
}

bool cvVec2f::greatOrEqual(const cvVec2f& v1, const cvVec2f& v2)
{
	return cvVec2f::equal(v1, v2) || cvVec2f::greater(v1, v2);
}

cvVec2f cvVec2f::operator*(float s) const
{
    cvVec2f ret = *this;
    ret.setScale(s);
    return ret;
}

cvVec2f cvVec2f::operator/(float s) const
{
    cvAssert(std::abs(s) > CV_FLOAT_EPS);
    cvVec2f ret = *this;
    ret.setScale(1 / s);
    return ret;
}

cvVec2f cvVec2f::operator+(const cvVec2f& v2) const
{
    cvVec2f ret;
    ret.setAdd(*this, v2);
    return ret;
}

cvVec2f cvVec2f::operator-(const cvVec2f& v2) const
{
    cvVec2f ret;
    ret.setSub(*this, v2);
    return ret;
}

cvVec2f cvVec2f::operator-() const
{
    cvVec2f ret = *this;
    ret *= -1;
    return ret;
}

cvVec2f& cvVec2f::operator+=(const cvVec2f& v)
{
    setAdd(*this, v);
    return *this;
}

cvVec2f& cvVec2f::operator-=(const cvVec2f& v)
{
    setSub(*this, v);
    return *this;
}

cvVec2f& cvVec2f::operator*=(float s)
{
    setScale(s);
    return *this;
}

cvVec2f& cvVec2f::operator/=(float s)
{
    cvAssert(std::abs(s) > CV_FLOAT_EPS);
    setScale(1 / s);
    return *this;
}

inline cvVec2f& cvVec2f::operator=(const cvVec2f& v)
{
    x = v.x;
    y = v.y;
    return *this;
}

bool cvVec2f::operator==(const cvVec2f& v) const
{
    return abs(v.x - x) < CV_FLOAT_EPS && abs(v.y - y) < CV_FLOAT_EPS;
}

inline float cvVec3f::length() const
{
    float sl = sqrLength();
    return sqrtf(sl);
}

inline float cvVec3f::sqrLength() const
{
    return x * x + y * y + z * z;
}


inline void cvVec3f::add(const cvVec3f& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

inline void cvVec3f::setAdd(const cvVec3f& v1, const cvVec3f& v2)
{
    *this = v1;
    add(v2);
}


inline void cvVec3f::setNegate()
{
    setScale(-1);
}

inline void cvVec3f::sub(const cvVec3f& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
}

inline void cvVec3f::setSub(const cvVec3f& v1, const cvVec3f& v2)
{
    *this = v1;
    sub(v2);
}

// this = this + v * s
inline void cvVec3f::addMul(const cvVec3f& v, float s)
{
    x += v.x * s;
    y += v.y * s;
    z += v.z * s;
}


inline void cvVec3f::setScale(float s)
{
    x *= s;
    y *= s;
    z *= s;
}


inline float cvVec3f::dot(const cvVec3f& v1) const
{
    return x * v1.x + y * v1.y + z * v1.z;
}


inline void cvVec3f::normalize()
{
    float l = length();
    if(l == 0)
    {
        cvAssertMsg(false, "Normalize null vector");
        return;
    }
    setScale(1.0f / l);
}

inline cvVec3f cvVec3f::getNormalized() const
{
    cvVec3f ret = *this;
    ret.normalize();
    return ret;
}


/// project this to v
inline void cvVec3f::setProj(const cvVec3f& v)
{
	cvAssert(v.length() > CV_FLOAT_EPS);

	cvVec3f v1(v);
	v1.normalize();
	float d = dot(v1);
	x = v1.x * d;
	y = v1.y * d;
	z = v1.z * d;
}

inline cvVec3f cvVec3f::project(const cvVec3f& v) const
{
    auto ret = *this;
    ret.setProj(v);
    return ret;
}

// this cross v
inline cvVec3f cvVec3f::cross(const cvVec3f& v) const
{
    cvVec3f ret;
    ret.x = y * v.z - z * v.y;
    ret.y = z * v.x - x * v.z;
    ret.z = x * v.y - y * v.x;
    return ret;
}


// distance to v
inline float cvVec3f::distance(const cvVec3f& v) const
{
    cvVec3f t;
    t.setSub(*this, v);
    return t.length();
}

inline cvVec3f cvVec3f::min3(const cvVec3f& v1, const cvVec3f& v2)
{
    cvVec3f t;
    t.x = std::fmin(v1.x, v2.x);
    t.y = std::fmin(v1.y, v2.y);
    t.z = std::fmin(v1.z, v2.z);
    return t;
}

inline cvVec3f cvVec3f::max3(const cvVec3f& v1, const cvVec3f& v2)
{
    cvVec3f t;
    t.x = std::fmax(v1.x, v2.x);
    t.y = std::fmax(v1.y, v2.y);
    t.z = std::fmax(v1.z, v2.z);
    return t;
}


// v1 == v2 ? with epsilon
inline bool cvVec3f::equal(const cvVec3f& v1, const cvVec3f& v2)
{
	return almost_equal(v1.x, v2.x) && almost_equal(v1.y, v2.y);
}

// v1 < v2 ?
inline bool cvVec3f::less(const cvVec3f& v1, const cvVec3f& v2)
{
    return !equal(v1, v2) && v1.x < v2.x && v1.y < v2.y && v1.z < v2.z;
}

// v1 <= v2 ?
inline bool cvVec3f::lessOrEqual(const cvVec3f& v1, const cvVec3f& v2)
{
    return equal(v1, v2) || less(v1, v2);
}

// v2 > v1 ?
inline bool cvVec3f::greater(const cvVec3f& v1, const cvVec3f& v2)
{
    return !equal(v1, v2) && v1.x > v2.x && v1.y > v2.y && v1.z > v2.z;
}

// v2 >= v1 ?
inline  bool cvVec3f::greatOrEqual(const cvVec3f& v1, const cvVec3f& v2)
{
    return equal(v1, v2) || greater(v1, v2);
}

//  v1 = v * s
inline cvVec3f cvVec3f::operator*(float s) const
{
    cvVec3f t = *this;
    t.setScale(s);
    return t;
}

// v *= s
inline cvVec3f& cvVec3f::operator*=(float s)
{
    setScale(s);
    return *this;
}

// v1 = v / s
inline cvVec3f cvVec3f::operator/(float s) const
{
    cvVec3f v = *this;
    v /= s;
    return v;
}

// v /= s
inline cvVec3f& cvVec3f::operator/=(float s)
{
    if(abs(s) < CV_FLOAT_EPS)
    {
        cvAssertMsg(false, "div by 0");
        return *this;
    }
    setScale(1.0f / s);
    return *this;
}

// v1 = v + v2
inline cvVec3f cvVec3f::operator+(const cvVec3f& v2) const
{

    cvVec3f ret = *this;
    ret.add(v2);
    return ret;
}

// v1 += v2
inline cvVec3f& cvVec3f::operator+=(const cvVec3f& v2)
{
    add(v2);
    return *this;
}

// v1 = v - v2
inline cvVec3f cvVec3f::operator-(const cvVec3f& v2) const
{
    cvVec3f ret = *this;
    ret.sub(v2);
    return ret;
}

// v1 -= v2
inline cvVec3f& cvVec3f::operator-=(const cvVec3f& v2)
{
    sub(v2);
    return *this;
}

// unary minus
inline cvVec3f cvVec3f::operator-() const
{
    cvVec3f ret = *this;
    ret.setNegate();
    return ret;
}

inline bool cvVec3f::operator==(const cvVec3f& v) const
{
    return std::abs(v.x - x) < CV_FLOAT_EPS && std::abs(v.y - y) < CV_FLOAT_EPS
        && std::abs(v.z - z) < CV_FLOAT_EPS;
}

