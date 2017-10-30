#include "cvWorld.h"

cvBodyId cvWorld::createBody(const cvBodyCInfo& cInfo, bool addBody)
{
    cvBodyId id = m_bodyBuffer.alloc(cInfo);
    cvBody& body = m_bodyBuffer.getAt(id);


    return id;
}

void cvWorld::addBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyBuffer.getAt(bodyId);
    cvAabb aabb;
    body.getAabb(aabb);
    m_broadPhase->addBody(body);
}

void cvWorld::removeBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyBuffer.getAt(bodyId);
    cvAabb aabb;
    m_broadPhase->removeBody(body);
}
