#include "cvMath.h"

void cvTransform::setIdentity()
{
	m_Rotation = 0;
	m_Translation.set(0, 0);
}

void cvTransform::toMat33(cvMat33& outMat) const
{
	outMat.setIdentity();
	outMat.setRotationDeg(m_Rotation);
	outMat.setTranslation(m_Translation);
}