#include "cvBodyManager.h"

cvBodyManager::cvBodyManager()
{
}

cvBodyId cvBodyManager::allocate(const cvBodyCInfo& cinfo)
{
    cvBodyId id = m_bodyBuffer.alloc(cinfo);
    m_allocated.insert(id);
    return id;
}

void cvBodyManager::free(cvBodyId bodyId)
{
    m_bodyBuffer.free(bodyId);
    m_allocated.erase(bodyId);
}

const cvBody& cvBodyManager::getBody(cvBodyId bodyId) const
{
    return m_bodyBuffer.getAt(bodyId);
}

cvBody& cvBodyManager::accessBody(cvBodyId bodyId)
{
    return m_bodyBuffer.accessAt(bodyId);
}
