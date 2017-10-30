#include "cvBody.h"

cvBody::cvBody()
{
}

cvBody::cvBody(cvBodyCInfo& cinfo)
{
	m_transform = cinfo.m_initTransform;
	m_shape = cinfo.m_shape;
	m_mass.m_x = cinfo.m_mass;
}

void cvBody::getAabb(cvAabb& out) const
{
    m_shape->getAabb(m_transform, out);
}


