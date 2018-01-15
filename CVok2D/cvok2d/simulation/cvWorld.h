#pragma once
#include "cvBroadphase.h"
#include <core/collection/cvFreeList.h>
#include "cvBody.h"
#include <dyanmic/cvMotion.h>
#include <simulation/cvBodyManager.h>
#include <simulation/cvMotionManager.h>

struct cvWorldCInfo
{
    std::shared_ptr<cvShape> m_shape;
    cvTransform m_initTrans;
};

class cvWorld
{
public:
    typedef cvFreeList<cvMotion, cvMotionId> MotionBuffer;
public:
	cvWorld(cvWorldCInfo&  cinfo);

    cvBodyId createBody(const cvBodyCInfo& cInfo, bool addBody);
    void addBody(cvBodyId bodyId);
    void removeBody(cvBodyId bodyId);

    cvMotionId createMotion();

    const cvBodyManager& getBodyManager()const {return m_bodyManager;}

    cvBody& accessBody(cvBodyId id) {return m_bodyManager.accessBody(id);}
    const cvBody& getBody(cvBodyId id) const {return m_bodyManager.getBody(id);}

    void setBodyVelocity(cvBodyId bodyId, const cvVec2f& linVel, float angVel);
    void setBodyLinearVelocity(cvBodyId bodyId, const cvVec2f& linVel);
    void setBodyAngularVelocity(cvBodyId bodyId, float angVel);

    cvVec2f getBodyLinearVelocity(cvBodyId bodyId) const;
    float getBodyAngluarVelocity(cvBodyId bodyId) const;

    cvMotion& accessBodyMotion(cvBodyId bodyId);
    const cvMotion& getBodyMotion(cvBodyId bodyId) const;

    // Simulation related
public:
    void integrate(float dt);
private:
	cvBroadphase* m_broadPhase;

    cvBodyManager m_bodyManager;
    cvMotionManager m_motionManager;
};
