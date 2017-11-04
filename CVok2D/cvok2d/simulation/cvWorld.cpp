#include "cvWorld.h"
#include <simulation/cvBroadPhaseSAP.h>

cvWorld::cvWorld(cvWorldCInfo&  cinfo)
{
    m_broadPhase = new cvBroadphaseSAP(cvBroadphaseCInfo());
}

cvBodyId cvWorld::createBody(const cvBodyCInfo& cInfo, bool addBody)
{
    cvBodyId id = m_bodyManager.allocate(cInfo);
    if(addBody)
    {
        m_broadPhase->addBody(m_bodyManager.accessBody(id));
    }

    return id;
}

void cvWorld::addBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyManager.accessBody(bodyId);
    cvAabb aabb;
    body.getAabb(aabb);
    m_broadPhase->addBody(body);
}

void cvWorld::removeBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyManager.accessBody(bodyId);
    cvAabb aabb;
    m_broadPhase->removeBody(body);
}

void cvWorld::integrate(float dt)
{
    auto iter = m_bodyManager.getBodyIter();
    while(iter.isValid())
    {
        cvBody& body = m_bodyManager.accessBody(*iter);
    }
}
