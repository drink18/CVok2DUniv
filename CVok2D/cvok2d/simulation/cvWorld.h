#pragma once
#include "cvBroadphase.h"
#include <core/collection/cvFreeList.h>
#include "cvBody.h"
#include <dyanmic/cvMotion.h>

struct cvWorldCInfo
{
    std::shared_ptr<cvShape> m_shape;
    cvTransform m_initTrans;
};

class cvWorld
{
public:

    typedef cvFreeList<cvBody, cvBodyId> BodyBuffer;
    typedef cvFreeList<cvMotion, cvMotionId> MotionBuffer;
public:
	cvWorld(cvWorldCInfo&  cinfo)
	{}

    cvBodyId createBody(const cvBodyCInfo& cInfo, bool addBody);
    void addBody(cvBodyId bodyId);
    void removeBody(cvBodyId bodyId);

    cvMotionId createMotion();

    const BodyBuffer& getBodyBuffer() const {return m_bodyBuffer;}
private:
	cvBroadphase* m_broadPhase;

    BodyBuffer  m_bodyBuffer;
    MotionBuffer m_motionBuffer;
};
