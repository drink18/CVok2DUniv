#include "cvWorld.h"
#include <world/cvBroadPhaseSAP.h>

using namespace std;

cvWorld::cvWorld(cvWorldCInfo&  cinfo)
    :m_cInfo(cinfo)
{
    m_broadPhase = cinfo.m_broadPhase;
    if(!m_broadPhase)
    {
        cvBroadphaseCInfo binfo;
        binfo.m_AABBExpansion = cinfo.m_bpAABBExpesnion;
        m_broadPhase = new cvBroadphaseSAP(binfo);
    }
}

cvBodyId cvWorld::createBody(const cvBodyCInfo& cInfo, bool add)
{
    cvBodyId id = m_bodyManager.allocate(cInfo);

    cvBody& body = m_bodyManager.accessBody(id);
	body.m_transform = cInfo.m_initTransform;
	body.m_shape = cInfo.m_shape;
	body.m_mass.x = cInfo.m_mass;
    cvMotion motion;
    body.m_motionId = m_motionManager.addMotion(motion);
    body.m_id = id;

    if(add)
    {
        addBody(id);
    }

    return id;
}

void cvWorld::addBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyManager.accessBody(bodyId);
    m_broadPhase->addBody(body);
}

void cvWorld::removeBody(cvBodyId bodyId)
{
    cvBody& body = m_bodyManager.accessBody(bodyId);
    m_broadPhase->removeBody(body);
}

void cvWorld::integrate(float dt)
{
    for(auto iter = m_bodyManager.getBodyIter();iter.isValid(); ++iter)
    {
        cvBody& body = m_bodyManager.accessBody(*iter);
        const cvMotion& motion = m_motionManager.getMotion(body.getMotionId());
        cvTransform& xform = body.accessTransform();

        xform.m_Rotation += motion.m_angularVel * dt;
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

void cvWorld::setBodyTransform(cvBodyId id, const cvTransform& newTrans)
{
    cvBody& body = m_bodyManager.accessBody(id);
    body.m_transform = newTrans;

    if(body.getBroadphaseHandle().isValid())
    {
        m_broadPhase->markBodyDirty(body);
    }
}

cvTransform cvWorld::getBodyTransform(cvBodyId id) const
{
    auto& body = m_bodyManager.getBody(id);
    return body.m_transform;
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


void cvWorld::simulate(const cvSimInfo& info)
{
    // for now mark all bodies dirty
    auto iter = m_bodyManager.getBodyIter();
    while(iter.isValid())
    {
        auto& body = getBody(*iter);
        m_broadPhase->markBodyDirty(body);
        iter++;
    }

    //update dirty aabb nodes
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> removedPairs;
    m_broadPhase->updateDirtyNodes(newPairs, removedPairs);

    integrate(info.deltaTime);
}

void cvWorld::getBodyBPAabb(cvBodyId bodyId, cvAabb& outAabb) const
{
   const cvBody& body = getBody(bodyId);
   cvAssert(body.getBroadphaseHandle().isValid());
   m_broadPhase->getBpAABB(body.getBroadphaseHandle(), outAabb);
}
