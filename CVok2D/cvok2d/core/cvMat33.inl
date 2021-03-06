#pragma once

#include "cvMath.h"
#include "cvMathDefs.h"
#include <cmath>

void cvMat33::setIdentity()
{
	set(1, 0, 0,
		0, 1, 0,
		0, 0, 1);
}

void cvMat33::setTranslation(const cvVec2f& v)
{
	m_cols[0].z = v.x;
	m_cols[1].z = v.y;
}

void cvMat33::setRotationDeg(float angleDeg)
{
    setRotation(DEG2RAD(angleDeg));
}

void cvMat33::setRotation(float angle)
{
	float cosA = cos(angle);
	float sinA = sin(angle);

	m_cols[0].x = cosA;
	m_cols[0].y = -sinA;
	m_cols[1].x = sinA;
	m_cols[1].y = cosA;
}

void cvMat33::mul(cvVec2f& v) const
{
	const float x = m_cols[0].x * v.x + m_cols[0].y * v.y;
	const float y = m_cols[1].x * v.x + m_cols[1].y * v.y;
	v.x = x;
	v.y = y;
}

void cvMat33::mul(cvVec3f& v) const
{
	const float x = m_cols[0].x * v.x + m_cols[0].y * v.y + m_cols[0].z * v.z;
	const float y = m_cols[1].x * v.x + m_cols[1].y * v.y + m_cols[1].z * v.z;
	const float z = m_cols[2].x * v.x + m_cols[2].y * v.y + m_cols[2].z * v.z;
	v.x = x;
	v.y = y;
	v.z = z;
}

void cvMat33::setMul(const cvMat33& m)
{
	cvMat33 tm(*this);
	tm.mul(m, *this);
}

void cvMat33::mul(const cvMat33& m, cvMat33& om) const
{
	om.m_cols[0].x = m_cols[0].x * m.m_cols[0].x + m_cols[1].x * m.m_cols[0].y + m_cols[2].x * m.m_cols[0].z;
	om.m_cols[1].x = m_cols[0].x * m.m_cols[1].x + m_cols[1].x * m.m_cols[1].y + m_cols[2].x * m.m_cols[1].z;
	om.m_cols[2].x = m_cols[0].x * m.m_cols[2].x + m_cols[1].x * m.m_cols[2].y + m_cols[2].x * m.m_cols[2].z;

	om.m_cols[0].y = m_cols[0].y * m.m_cols[0].x + m_cols[1].y * m.m_cols[0].y + m_cols[2].y * m.m_cols[0].z;
	om.m_cols[1].y = m_cols[0].y * m.m_cols[1].x + m_cols[1].y * m.m_cols[1].y + m_cols[2].y * m.m_cols[1].z;
	om.m_cols[2].y = m_cols[0].y * m.m_cols[2].x + m_cols[1].y * m.m_cols[2].y + m_cols[2].y * m.m_cols[2].z;

	om.m_cols[0].z = m_cols[0].z * m.m_cols[0].x + m_cols[1].z * m.m_cols[0].y + m_cols[2].z * m.m_cols[0].z;
	om.m_cols[1].z = m_cols[0].z * m.m_cols[1].x + m_cols[1].z * m.m_cols[1].y + m_cols[2].z * m.m_cols[1].z;
	om.m_cols[2].z = m_cols[0].z * m.m_cols[2].x + m_cols[1].z * m.m_cols[2].y + m_cols[2].z * m.m_cols[2].z;
}

void cvMat33::transformVector(cvVec2f& v) const
{
	mul(v);
}

void cvMat33::transformPoint(cvVec2f& v) const 
{
	mul(v);
	v.x += m_cols[0].z;
	v.y += m_cols[1].z;
}

void cvMat33::transformVector(const cvVec2f& v, cvVec2f& ov) const 
{
	ov.x = v.x; ov.y = v.y;
	transformVector(ov);
}

void cvMat33::transformPoint(const cvVec2f& v, cvVec2f& ov) const
{
	ov.x = v.x; ov.y = v.y;
	transformPoint(ov);
}

cvVec2f cvMat33::operator*(const cvVec2f& v) const
{
    cvVec2f ret;
    transformPoint(v, ret);
    return ret;
}

cvMat33 cvMat33::operator*(const cvMat33& m) const
{
    cvMat33 ret = *this;
    ret.setMul(m);
    return ret;
}

cvMat33& cvMat33::operator*=(const cvMat33& m)
{
    setMul(m);
    return *this;
}

float cvMat33::getDet() const
{
    float det = m_cols[0].x * m_cols[1].y * m_cols[2].z
        + m_cols[0].y * m_cols[1].z * m_cols[2].x
        + m_cols[0].z * m_cols[1].x * m_cols[2].y
        - m_cols[0].z * m_cols[1].y * m_cols[2].x
        - m_cols[0].y * m_cols[1].x * m_cols[2].z
        - m_cols[0].x * m_cols[1].z * m_cols[2].y;
    return det;
}

void cvMat33::getInvert(cvMat33& om) const
{

    float det = getDet();
    if(abs(det) < CV_FLOAT_EPS)
        cvAssertMsg(false, "Matrix not invertable");
    else
    {
        om.m_cols[0].x =  m_cols[1].y * m_cols[2].z - m_cols[1].z * m_cols[2].y;
        om.m_cols[1].x = -m_cols[1].x * m_cols[2].z + m_cols[1].z * m_cols[2].x;
        om.m_cols[2].x = m_cols[1].x * m_cols[2].y - m_cols[1].y * m_cols[2].x;

        om.m_cols[0].y = -m_cols[0].y * m_cols[2].z + m_cols[0].z * m_cols[2].y;
        om.m_cols[1].y = m_cols[0].x * m_cols[2].z - m_cols[0].z * m_cols[2].x;
        om.m_cols[2].y = -m_cols[0].x * m_cols[2].y + m_cols[0].y * m_cols[2].x;

        om.m_cols[0].z =  m_cols[0].y * m_cols[1].z - m_cols[0].z * m_cols[1].y;
        om.m_cols[1].z = -m_cols[0].x * m_cols[1].z + m_cols[0].z * m_cols[1].x;
        om.m_cols[2].z =  m_cols[0].x * m_cols[1].y - m_cols[0].y * m_cols[1].x;

        for(int c = 0; c < 3; c++)
        {
            om.m_cols[c].x /= det;
            om.m_cols[c].y /= det;
            om.m_cols[c].z /= det;
        }
    }
}
