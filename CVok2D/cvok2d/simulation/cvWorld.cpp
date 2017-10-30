#include "cvWorld.h"

cvBodyId cvWorld::createBody(const cvBodyCInfo& cInfo, bool addBody)
{
    cvBodyId id = m_bodyBuffer.alloc(cInfo);
    cvBody& body = m_bodyBuffer.getAt(id);


    return id;
}

void cvWorld::addBody(cvBodyId bodyId)
{
    const cvBody& body = m_bodyBuffer.getAt(bodyId);
    cvAabb aabb;
    body.getAabb(aabb);
    cvBroadphaseHandle handle = m_broadPhase->addNode(aabb);
}
