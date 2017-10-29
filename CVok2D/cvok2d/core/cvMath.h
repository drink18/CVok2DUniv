#pragma once

#include <cmath>
#include "cvCore.h"
#include "cvMathDefs.h"

#define cvSqrt sqrtf


inline bool almost_equal(float x, float y)
{
	return std::abs(x - y) < CV_FLOAT_EPS;
}

class cvVec2f
{
public:
	float m_x;
	float m_y;

public:
	cvVec2f(float x, float y) : m_x(x), m_y(y){}
	cvVec2f() : m_x(0), m_y(0) {}
	cvVec2f(const cvVec2f& v) : m_x(v.m_x), m_y(v.m_y) {}
	void set(float x, float y)
	{
		m_x = x;
		m_y = y;
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

    // v1 -= v2
    inline cvVec2f& operator-=(const cvVec2f& v2);
};

class cvVec3f
{
public:
	cvVec3f() {};
	cvVec3f(float x, float y, float z) : m_x(x), m_y(y), m_z(z) {}
	cvVec3f(const cvVec3f& v) : m_x(v.m_x), m_y(v.m_y), m_z(v.m_z) {}
	void set(const cvVec3f& v)
	{
		m_x = v.m_x;
		m_y = v.m_y;
		m_z = v.m_z;
	}

public:
	float m_x;
	float m_y;
	float m_z;
};

// column major 3x3 matrix
class cvMat33
{
public:
	cvMat33() {}
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
		m_cols[0].m_x = m00; m_cols[0].m_y = m10; m_cols[0].m_z = m20;
		m_cols[1].m_x = m01; m_cols[1].m_y = m11; m_cols[1].m_z = m21;
		m_cols[2].m_x = m02; m_cols[2].m_y = m12; m_cols[2].m_z = m22;
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
public:
	cvVec2f m_Translation;
	float m_Rotation;
};


#include "cvVec.inl"
#include "cvMat33.inl"
#include "cvTransform.inl"
