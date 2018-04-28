#pragma once

#include <core/collection/cvFreeList.h>
#include <dyanmic/cvMotion.h>
#include <unordered_set>

class cvMotionManager
{
public:
    cvMotionManager();
    ~cvMotionManager();

    cvMotion& accessMotion(cvMotionId id);
    const cvMotion& getMotion(cvMotionId id) const;

    cvMotionId addMotion(const cvMotion& motion);
    void removeMotion(cvMotionId id);

    void refreshSolverId();

    const std::unordered_set<cvMotionId>& getAllocatedIds() const {return  m_allocated;}

private:
    cvFreeList<cvMotion, cvMotionId> m_motionBuffer;
    std::unordered_set<cvMotionId> m_allocated;
};
