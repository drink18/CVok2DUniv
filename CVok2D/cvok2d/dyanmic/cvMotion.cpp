#include "cvMotion.h"
#include <world/cvBody.h>

cvMotionId const cvMotion::StaticMotionId = cvMotionId(0);

void cvMotion::InitializeMotion(cvMotion& motion, cvMotion::MotionType mt, const cvBodyCInfo& cinfo)
{
    cvAssertMsg(mt != cvMotion::MotionType::Static, "Should not be called on static bodies");
    motion.m_transform = cinfo.m_initTransform;
    motion.m_angularVel = 0;
    motion.m_linearVel = cvVec2f::getZero();

    motion.m_invMassAndInertia = cvVec2f::getZero();
    if(mt == cvMotion::MotionType::Dynamic)
    {
        float invMass = 1.0f / cinfo.m_mass;
        float invInertia = 1;
        motion.m_invMassAndInertia.set(invMass, invInertia);
    }
}

