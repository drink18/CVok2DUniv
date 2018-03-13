#pragma once

#include <core/collection/cvFreeList.h>
#include <dyanmic/cvMotion.h>

class cvMotionManager
{
public:
    cvMotionManager();
    ~cvMotionManager();

    cvMotion& accessMotion(cvMotionId id);
    const cvMotion& getMotion(cvMotionId id) const;

    cvMotionId addMotion(const cvMotion& motion);
    void removeMotion(cvMotionId id);

private:
    cvFreeList<cvMotion, cvMotionId> m_motionBuffer;
};
