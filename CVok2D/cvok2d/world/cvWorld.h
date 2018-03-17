#pragma once
#include "cvBroadphase.h"
#include <core/collection/cvFreeList.h>
#include "cvBody.h"
#include <dyanmic/cvMotion.h>

#include <world/cvBodyManager.h>
#include <world/cvMotionManager.h>
#include <core/cvAabb.h>

struct cvWorldCInfo
{
    cvBroadphase* m_broadPhase = nullptr;
    float m_bpAABBExpesnion = 0.05f;
};

struct cvSimInfo
{
    float deltaTime;
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

    void setBodyTransform(cvBodyId id, const cvTransform& newTrans);
    cvTransform getBodyTransform(cvBodyId id) const;

    void setBodyVelocity(cvBodyId bodyId, const cvVec2f& linVel, float angVel);
    void setBodyLinearVelocity(cvBodyId bodyId, const cvVec2f& linVel);
    void setBodyAngularVelocity(cvBodyId bodyId, float angVel);

    cvVec2f getBodyLinearVelocity(cvBodyId bodyId) const;
    float getBodyAngluarVelocity(cvBodyId bodyId) const;

    cvMotion& accessBodyMotion(cvBodyId bodyId);
    const cvMotion& getBodyMotion(cvBodyId bodyId) const;

    const cvBroadphase& getBroadphase() const { return *m_broadPhase; }
    void getBodyBPAabb(cvBodyId bodyId, cvAabb& outAabb) const;

    // Simulation related
public:
    void simulate(const cvSimInfo& info);
    void integrate(float dt);

private:
    cvWorldCInfo m_cInfo;
	cvBroadphase* m_broadPhase = nullptr;

    cvBodyManager m_bodyManager;
    cvMotionManager m_motionManager;
};
