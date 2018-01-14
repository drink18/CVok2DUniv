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

