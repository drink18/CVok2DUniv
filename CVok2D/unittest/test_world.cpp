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
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05));

    cvBodyId bodyId = world->createBody(bodyCInfo, true);
    world->removeBody(bodyId);

    delete world;
}

