#include "cvBody.h"

cvBody::cvBody()
{
    m_flags = 0;
}

cvBody::cvBody(cvBodyCInfo& cinfo)
{
	m_transform = cinfo.m_initTransform;
	m_shape = cinfo.m_shape;
	m_mass.x = cinfo.m_mass;

}

void cvBody::getAabb(cvAabb& out) const
{
    m_shape->getAabb(m_transform, out);
}

void cvBody::setFlag(cvBody::Flags flags, bool enable)
{
    if(enable)
        m_flags |= flags;
    else
        m_flags &= (~flags);
}
