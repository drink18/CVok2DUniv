#pragma once
#include "cvBroadphase.h"
#include <core/collection/cvFreeList.h>
#include "cvBody.h"
#include <dyanmic/cvMotion.h>
#include <simulation/cvBodyManager.h>

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

    // Simulation related
protected:
    void integrate(float dt);
private:
	cvBroadphase* m_broadPhase;

    cvBodyManager m_bodyManager;
    MotionBuffer m_motionBuffer;
};
