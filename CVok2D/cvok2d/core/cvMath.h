#pragma once

#include <cmath>
#include "cvCore.h"
#include "cvMathDefs.h"
#include <ostream>

#define cvSqrt sqrtf


inline bool almost_equal(float x, float y)
{
	return std::abs(x - y) < CV_FLOAT_EPS;
}

class cvVec2f
{
public:
	float x;
	float y;

public:
	cvVec2f(float _x, float _y) : x(_x), y(_y){}
	cvVec2f() : x(0), y(0) {}
	cvVec2f(const cvVec2f& v) : x(v.x), y(v.y) {}
	void set(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	inline float length() const;
	inline float sqrLength() const;

	inline void add(const cvVec2f& v);
	inline void setAdd(const cvVec2f& v1, const cvVec2f& v2);

	inline void setNegate();
	inline void setSub(const cvVec2f& v1, const cvVec2f& v2);
	inline void sub(const cvVec2f& v);

    // this = this + v * s
    inline void addMul(const cvVec2f& v, float s);

	inline void setScale(float s);

	inline float dot(const cvVec2f& v1) const;

	inline void normalize();
	inline cvVec2f getNormalized() const;

	/// project this to v
	inline void setProj(const cvVec2f& v);
	inline cvVec2f project(const cvVec2f& v) const;

	// this cross v
	inline float cross(const cvVec2f& v) const;

	// distance to v
	inline float distance(const cvVec2f& v) const;

	static inline cvVec2f min2(const cvVec2f& v1, const cvVec2f& v2);
	static inline cvVec2f max2(const cvVec2f& v1, const cvVec2f& v2);

	// v1 == v2 ? with epsilon
	inline static bool equal(const cvVec2f& v1, const cvVec2f& v2);

	// v1 < v2 ?
	inline static bool less(const cvVec2f& v1, const cvVec2f& v2);
	// v1 <= v2 ?
	inline static bool lessOrEqual(const cvVec2f& v1, const cvVec2f& v2);

	// v2 > v1 ? 
	inline static bool greater(const cvVec2f& v1, const cvVec2f& v2);
	// v2 >= v1 ? 
	inline static bool greatOrEqual(const cvVec2f& v1, const cvVec2f& v2);


    ////////////////////////////////////////////////////////////////////
    // overloaded operators
    // ///////////////////////////////////////////////////////////////
    
    //  v1 = v * s
    inline cvVec2f operator*(float s) const;

    // v *= s
    inline cvVec2f& operator*=(float s);
    
    // v1 = v / s
    inline cvVec2f operator/(float s) const;

    // v /= s
    inline cvVec2f& operator/=(float s);

    // v1 = v + v2
    inline cvVec2f operator+(const cvVec2f& v2) const;

    // v1 += v2
    inline cvVec2f& operator+=(const cvVec2f& v2);

    // v1 = v - v2
    inline cvVec2f operator-(const cvVec2f& v2) const;

    // unary minus
    inline cvVec2f operator-() const;

    // v1 -= v2
    inline cvVec2f& operator-=(const cvVec2f& v2);

    inline cvVec2f& operator=(const cvVec2f& v);

    inline bool operator==(const cvVec2f& v) const;

    static cvVec2f getZero();
};

class cvVec3f
{
public:
	cvVec3f() {};
	cvVec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	cvVec3f(const cvVec3f& v) : x(v.x), y(v.y), z(v.z) {}
	void set(const cvVec3f& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	inline float length() const;
	inline float sqrLength() const;

	inline void add(const cvVec3f& v);
	inline void setAdd(const cvVec3f& v1, const cvVec3f& v2);

	inline void setNegate();
	inline void setSub(const cvVec3f& v1, const cvVec3f& v2);
	inline void sub(const cvVec3f& v);

    // this = this + v * s
    inline void addMul(const cvVec3f& v, float s);

	inline void setScale(float s);

	inline float dot(const cvVec3f& v1) const;

	inline void normalize();
	inline cvVec3f getNormalized() const;

	/// project this to v
	inline void setProj(const cvVec3f& v);
	inline cvVec3f project(const cvVec3f& v) const;

	// this cross v
	inline cvVec3f cross(const cvVec3f& v) const;

	// distance to v
	inline float distance(const cvVec3f& v) const;

	static inline cvVec3f min3(const cvVec3f& v1, const cvVec3f& v2);
	static inline cvVec3f max3(const cvVec3f& v1, const cvVec3f& v2);

	// v1 == v2 ? with epsilon
	inline static bool equal(const cvVec3f& v1, const cvVec3f& v2);

	// v1 < v2 ?
	inline static bool less(const cvVec3f& v1, const cvVec3f& v2);
	// v1 <= v2 ?
	inline static bool lessOrEqual(const cvVec3f& v1, const cvVec3f& v2);

	// v2 > v1 ? 
	inline static bool greater(const cvVec3f& v1, const cvVec3f& v2);
	// v2 >= v1 ? 
	inline static bool greatOrEqual(const cvVec3f& v1, const cvVec3f& v2);

    ////////////////////////////////////////////////////////////////////
    // overloaded operators
    // ///////////////////////////////////////////////////////////////

    //  v1 = v * s
    inline cvVec3f operator*(float s) const;

    // v *= s
    inline cvVec3f& operator*=(float s);

    // v1 = v / s
    inline cvVec3f operator/(float s) const;

    // v /= s
    inline cvVec3f& operator/=(float s);

    // v1 = v + v2
    inline cvVec3f operator+(const cvVec3f& v2) const;

    // v1 += v2
    inline cvVec3f& operator+=(const cvVec3f& v2);

    // v1 = v - v2
    inline cvVec3f operator-(const cvVec3f& v2) const;

    // unary minus
    inline cvVec3f operator-() const;

    // v1 -= v2
    inline cvVec3f& operator-=(const cvVec3f& v2);

    inline bool operator==(const cvVec3f& v2) const;

public:
	float x;
	float y;
	float z;
};

// column major 3x3 matrix
class cvMat33
{
public:
	cvMat33() {setIdentity();}
	cvMat33(const cvMat33& o) 
	{
		m_cols[0].set(o.m_cols[0]);
		m_cols[1].set(o.m_cols[1]);
		m_cols[2].set(o.m_cols[2]);
	}

	inline void setIdentity();
	void set(float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22)
	{
		m_cols[0].x = m00; m_cols[0].y = m10; m_cols[0].z = m20;
		m_cols[1].x = m01; m_cols[1].y = m11; m_cols[1].z = m21;
		m_cols[2].x = m02; m_cols[2].y = m12; m_cols[2].z = m22;
	}

	inline void setTranslation(const cvVec2f& t);
	inline void setRotationDeg(float angleDeg);
	inline void setRotation(float angleDeg);

	inline void mul(cvVec3f& v) const;
	inline void mul(cvVec2f& v) const;
	// this *= m
	inline void setMul(const cvMat33& m);
	// om = this * m
	inline void mul(const cvMat33& m, cvMat33& om) const;

	inline void transformVector(cvVec2f& v) const;
	inline void transformPoint(cvVec2f& v) const;
	inline void transformVector(const cvVec2f& v, cvVec2f& ov) const;
	inline void transformPoint(const cvVec2f& v, cvVec2f& ov) const;

    inline void getInvert(cvMat33& om) const;
    inline float getDet() const;

    /////////////////////////////////////////////////////////////////////
    // Overloaded operators
    /////////////////////////////////////////////////////////////////////
    // transform vertex
    inline cvVec2f operator*(const cvVec2f& v) const;
 
    // matrix multiplication 
    // ret = this * m
    inline cvMat33 operator*(const cvMat33& m) const;

    // this *= m
    inline cvMat33& operator*=(const cvMat33& m);
public:
	cvVec3f m_cols[3];

};

class cvTransform
{
public:
	cvTransform() { setIdentity(); }
	
	cvTransform(const cvVec2f& translation, float angle)
		: m_Translation(translation), m_Rotation(angle) {}

	cvTransform(const cvTransform& other)
		: m_Translation(other.m_Translation), m_Rotation(other.m_Rotation) {}

	// convert to matrix 33
	inline void toMat33(cvMat33& outMat) const;
	inline void setIdentity();
    static cvTransform getIdentity() {return cvTransform();}
public:
	cvVec2f m_Translation;
	float m_Rotation;
};

inline void PrintTo(const cvVec2f& v, ::std::ostream* os)
{
    *os << "(" << v.x << ", " << v.y << ")";
}

inline void PrintTo(const cvVec3f& v, ::std::ostream* os)
{
    *os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

#include "cvVec.inl"
#include "cvMat33.inl"
#include "cvTransform.inl"
