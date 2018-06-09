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
    cvAssertMsg(id.isValid(), "invalid motionId");
    return m_motionBuffer.accessAt(id);
}

const cvMotion& cvMotionManager::getMotion(cvMotionId id) const
{
    cvAssertMsg(id.isValid(), "invalid motionId");
    return m_motionBuffer.getAt(id);
}

cvMotionId cvMotionManager::addMotion(const cvMotion& motion)
{
    cvMotionId id = m_motionBuffer.alloc(motion);
    m_allocated.insert(id);
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
    m_allocated.erase(id);
}

void cvMotionManager::refreshSolverId()
{
    int i = 1; // starting from 1 because solver body Id 0 is for all static bodies
    for(const auto& id : m_allocated)
    {
        cvMotion& m = accessMotion(id);
        m.solverBodyId = i++;
    }
}
