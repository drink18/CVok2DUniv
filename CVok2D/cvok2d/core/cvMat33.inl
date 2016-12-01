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
	m_cols[0].m_z = v.m_x;
	m_cols[1].m_z = v.m_y;
}

void cvMat33::setRotationDeg(float angleDeg)
{
	float cosA = cos(DEG2RAD(angleDeg));
	float sinA = sin(DEG2RAD(angleDeg));

	m_cols[0].m_x = cosA;
	m_cols[0].m_y = -sinA;
	m_cols[1].m_x = sinA;
	m_cols[1].m_y = cosA;
}

void cvMat33::mul(cvVec2f& v) const
{
	const float x = m_cols[0].m_x * v.m_x + m_cols[0].m_y * v.m_y;
	const float y = m_cols[1].m_x * v.m_x + m_cols[1].m_y * v.m_y;
	v.m_x = x;
	v.m_y = y;
}

void cvMat33::mul(cvVec3f& v) const
{
	const float x = m_cols[0].m_x * v.m_x + m_cols[0].m_y * v.m_y + m_cols[0].m_z * v.m_z;
	const float y = m_cols[1].m_x * v.m_x + m_cols[1].m_y * v.m_y + m_cols[1].m_z * v.m_z;
	const float z = m_cols[2].m_x * v.m_x + m_cols[2].m_y * v.m_y + m_cols[2].m_z * v.m_z;
	v.m_x = x;
	v.m_y = y;
	v.m_z = z;
}

void cvMat33::setMul(const cvMat33& m)
{
	cvMat33 tm(*this);
	tm.mul(m, *this);
}

void cvMat33::mul(const cvMat33& m, cvMat33& om) const
{
	om.m_cols[0].m_x = m_cols[0].m_x * m.m_cols[0].m_x + m_cols[1].m_x * m.m_cols[0].m_y + m_cols[2].m_x * m.m_cols[0].m_z;
	om.m_cols[1].m_x = m_cols[0].m_x * m.m_cols[1].m_x + m_cols[1].m_x * m.m_cols[1].m_y + m_cols[2].m_x * m.m_cols[1].m_z;
	om.m_cols[2].m_x = m_cols[0].m_x * m.m_cols[2].m_x + m_cols[1].m_x * m.m_cols[2].m_y + m_cols[2].m_x * m.m_cols[2].m_z;

	om.m_cols[0].m_y = m_cols[0].m_y * m.m_cols[0].m_x + m_cols[1].m_y * m.m_cols[0].m_y + m_cols[2].m_y * m.m_cols[0].m_z;
	om.m_cols[1].m_y = m_cols[0].m_y * m.m_cols[1].m_x + m_cols[1].m_y * m.m_cols[1].m_y + m_cols[2].m_y * m.m_cols[1].m_z;
	om.m_cols[2].m_y = m_cols[0].m_y * m.m_cols[2].m_x + m_cols[1].m_y * m.m_cols[2].m_y + m_cols[2].m_y * m.m_cols[2].m_z;

	om.m_cols[0].m_z = m_cols[0].m_z * m.m_cols[0].m_x + m_cols[1].m_z * m.m_cols[0].m_y + m_cols[2].m_z * m.m_cols[0].m_z;
	om.m_cols[1].m_z = m_cols[0].m_z * m.m_cols[1].m_x + m_cols[1].m_z * m.m_cols[1].m_y + m_cols[2].m_z * m.m_cols[1].m_z;
	om.m_cols[2].m_z = m_cols[0].m_z * m.m_cols[2].m_x + m_cols[1].m_z * m.m_cols[2].m_y + m_cols[2].m_z * m.m_cols[2].m_z;
}

void cvMat33::transformVector(cvVec2f& v) const
{
	mul(v);
}

void cvMat33::transformPoint(cvVec2f& v) const 
{
	mul(v);
	v.m_x += m_cols[0].m_z;
	v.m_y += m_cols[1].m_z;
}

void cvMat33::transformVector(const cvVec2f& v, cvVec2f& ov) const 
{
	ov.m_x = v.m_x; ov.m_y = v.m_y;
	transformVector(ov);
}

void cvMat33::transformPoint(const cvVec2f& v, cvVec2f& ov) const
{
	ov.m_x = v.m_x; ov.m_y = v.m_y;
	transformPoint(ov);
}
