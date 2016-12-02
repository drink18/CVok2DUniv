#include "cvBody.h"

cvBody::cvBody(cvBodyCInfo& cinfo)
{
	m_transform = cinfo.m_initTransform;
	m_shape = cinfo.m_shape;
	m_mass.m_x = cinfo.m_mass;
}


