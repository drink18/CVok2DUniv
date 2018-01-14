#include "cvMotionManager.h"
#include <core/cvCore.h>

cvMotionManager::cvMotionManager()
{
    cvMotion staticMotion;
    cvMotionId id = m_motionBuffer.alloc(staticMotion);
    cvAssertMsg(id == cvMotion::StaticMotionId, "");
}

cvMotionManager::~cvMotionManager()
{
}

cvMotion& cvMotionManager::accessMotion(cvMotionId id)
{
    return m_motionBuffer.accessAt(id);
}

const cvMotion& cvMotionManager::getMotion(cvMotionId id) const
{
    return m_motionBuffer.getAt(id);
}

cvMotionId cvMotionManager::addMotion(const cvMotion& motion)
{
    cvMotionId id = m_motionBuffer.alloc(motion);
    return id;
}

void cvMotionManager::removeMotion(cvMotionId id)
{
    if(id == cvMotion::StaticMotionId)
    {
        cvAssertMsg(false, "Cannot remove static motion");
        return;
    }
    m_motionBuffer.free(id);
}
