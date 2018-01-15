#include "gtest/gtest.h"

#include <simulation/cvWorld.h>
#include <shape/cvPolygonShape.h>

TEST(world, worldCreationDestruction)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    delete world;
}

TEST(world, add_remove_body)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId bodyId = world->createBody(bodyCInfo, true);
    world->removeBody(bodyId);

    delete world;
}

TEST(world, iterate_bodies)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);


    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    for(int i = 0; i < 100; ++i)
    {
        world->createBody(bodyCInfo, true);
    }


    auto& bodyManager = world->getBodyManager();
    auto bodyIter = bodyManager.getBodyIter();
    while(bodyIter.isValid())
    {
        cvBodyId bodyId = *bodyIter;
        const cvBody& body = bodyManager.getBody(bodyId);
        EXPECT_EQ(bodyCInfo.m_shape, body.getShape());
        bodyIter++;
    }
}

TEST(world, integrate)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);


    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(
            cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId id = world->createBody(bodyCInfo, true);
    world->setBodyAngularVelocity(id, 1.0f);
    world->setBodyLinearVelocity(id, cvVec2f(1.0f, 2.0f));
    world->integrate(0.01f);
    cvTransform xform = world->getBody(id).getTransform();

    EXPECT_NEAR(0.01f, xform.m_Rotation, CV_FLOAT_EPS);
    EXPECT_NEAR(0.01f, xform.m_Translation.x, CV_FLOAT_EPS);
    EXPECT_NEAR(0.02f, xform.m_Translation.y, CV_FLOAT_EPS);
}
