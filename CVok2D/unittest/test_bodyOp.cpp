#include "gtest/gtest.h"

#include <simulation/cvWorld.h>
#include <shape/cvPolygonShape.h>
#include <core/cvMath.h>

TEST(world, SetVel)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(
            cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId bodyId = world->createBody(bodyCInfo, true);
    world->setBodyVelocity(bodyId, cvVec2f(1, 2), 3);

    cvVec2f linVelBack = world->getBodyLinearVelocity(bodyId);
    EXPECT_EQ(cvVec2f(1.0f, 2.0f), linVelBack);

    float angVelBack = world->getBodyAngluarVelocity(bodyId);
    EXPECT_NEAR(3.0f, angVelBack, CV_FLOAT_EPS);

    delete world;
}

TEST(world, SetVelSep)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(
            cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId bodyId = world->createBody(bodyCInfo, true);
    cvVec2f linVelBack = world->getBodyLinearVelocity(bodyId);
    float angVelBack = world->getBodyAngluarVelocity(bodyId);
    EXPECT_EQ(cvVec2f(0.0f, 0.0f), linVelBack);
    EXPECT_NEAR(0, angVelBack, CV_FLOAT_EPS);

    world->setBodyLinearVelocity(bodyId, cvVec2f(1, 2));
    linVelBack = world->getBodyLinearVelocity(bodyId);
    EXPECT_EQ(cvVec2f(1.0f, 2.0f), linVelBack);

    world->setBodyAngularVelocity(bodyId, 3.0f);
    angVelBack = world->getBodyAngluarVelocity(bodyId);
    EXPECT_NEAR(3.0f, angVelBack, CV_FLOAT_EPS);
}

TEST(world, SetBodyTransform)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(
            cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId bodyId = world->createBody(bodyCInfo, true);

    cvTransform t;
    t.m_Translation.x = 1;
    t.m_Translation.y = 2;

    world->setBodyTransform(bodyId, t);

    cvTransform ut = world->getBodyTransform(bodyId);
    EXPECT_EQ(cvVec2f(1, 2), ut.m_Translation);
}
