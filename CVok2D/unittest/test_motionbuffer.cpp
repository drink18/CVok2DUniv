#include "gtest/gtest.h"

#include <simulation/cvMotionManager.h>

TEST(cvMotionManager, creation)
{
    cvMotionManager mgr;
    cvMotion motion;
}
/*
TEST(cvMotionManager, has_presets)
{
    cvMotionManager mgr;
    auto& motion = mgr.getMotion(cvMotion::StaticMotionId);
    cvVec2f zeroV;
    EXPECT_NEAR(0.0f, motion.getAngularVel(), CV_FLOAT_EPS);
    EXPECT_NEAR(0.0f, motion.getLinearVel().m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(0.0f, motion.getLinearVel().m_y, CV_FLOAT_EPS);
}

TEST(cvMotionManager, addMotion)
{
    cvMotionManager mgr;
    cvMotion motion;
    cvVec2f linVel(3, 4);
    motion.m_angularVel = 1;
    motion.m_linearVel = linVel;

    auto id = mgr.addMotion(motion);
    auto& retMo = mgr.getMotion(id);
    EXPECT_NEAR(linVel.m_x, retMo.m_linearVel.m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(linVel.m_y, retMo.m_linearVel.m_y, CV_FLOAT_EPS);
    EXPECT_EQ(1.0f, retMo.m_angularVel);
}

TEST(cvMotionManager, removeMotion)
{
    cvMotionManager mgr;
    cvMotion motion;
    auto id = mgr.addMotion(motion);
    mgr.removeMotion(id);
}
*/
