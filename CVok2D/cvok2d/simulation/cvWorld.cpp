#include "cvWorld.h"
#include <simulation/cvBroadPhaseSAP.h>

cvWorld::cvWorld(cvWorldCInfo&  cinfo)
{
    m_broadPhase = new cvBroadphaseSAP(cvBroadphaseCInfo());
}

cvBodyId cvWorld::createBody(const cvBodyCInfo& cInfo, bool addBody)
{
    cvBodyId id = m_bodyManager.allocate(cInfo);

    cvBody& body = m_bodyManager.accessBody(id);
	body.m_transform = cInfo.m_initTransform;
	body.m_shape = cInfo.m_shape;
	body.m_mass.x = cInfo.m_mass;
    cvMotion motion;
    body.m_motionId = m_motionManager.addMotion(motion);

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
    for(auto iter = m_bodyManager.getBodyIter();iter.isValid(); ++iter)
    {
        cvBody& body = m_bodyManager.accessBody(*iter);
        const cvMotion& motion = m_motionManager.getMotion(body.getMotionId());
        cvTransform& xform = body.accessTransform();

        xform.m_Rotation = motion.m_angularVel * dt;
        xform.m_Translation += motion.m_linearVel * dt;
    }
}

void cvWorld::setBodyVelocity(cvBodyId bodyId, const cvVec2f& linVel, float angVel)
{
    cvMotion& motion = accessBodyMotion(bodyId); 
    motion.m_linearVel = linVel;
    motion.m_angularVel = angVel;
}

void cvWorld::setBodyLinearVelocity(cvBodyId bodyId, const cvVec2f& linVel)
{
    cvMotion& motion = accessBodyMotion(bodyId); 
    motion.m_linearVel = linVel;
}

void cvWorld::setBodyAngularVelocity(cvBodyId bodyId, float angVel)
{
    cvMotion& motion = accessBodyMotion(bodyId); 
    motion.m_angularVel = angVel;
}

cvVec2f cvWorld::getBodyLinearVelocity(cvBodyId bodyId) const
{
    const cvMotion& motion = getBodyMotion(bodyId);
    return motion.m_linearVel;
}

float cvWorld::getBodyAngluarVelocity(cvBodyId bodyId) const
{
    const cvMotion& motion = getBodyMotion(bodyId);
    return motion.m_angularVel;
}

cvMotion& cvWorld::accessBodyMotion(cvBodyId bodyId)
{
    cvBody& body = m_bodyManager.accessBody(bodyId);
    cvMotionId motionId = body.getMotionId();
    return m_motionManager.accessMotion(motionId);
}

const cvMotion& cvWorld::getBodyMotion(cvBodyId bodyId) const
{
    const cvBody& body = m_bodyManager.getBody(bodyId);
    cvMotionId motionId = body.getMotionId();
    return m_motionManager.getMotion(motionId);
}
