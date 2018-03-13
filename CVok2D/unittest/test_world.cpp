#include "gtest/gtest.h"

#include <world/cvWorld.h>
#include <shape/cvPolygonShape.h>

class WorldTest : public ::testing::Test
{
protected:
    cvWorld* m_world = nullptr;

    virtual void SetUp()
    {
        cvWorldCInfo cInfo;
        m_world = new cvWorld(cInfo);
    }

    virtual void TearDown()
    {
        delete m_world;
    }

};

TEST(world, worldCreationDestruction)
{
    cvWorldCInfo cInfo;
    cvWorld* world = new cvWorld(cInfo);

    delete world;
}

TEST_F(WorldTest, add_remove_body)
{

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId bodyId = m_world->createBody(bodyCInfo, true);
    cvBody& body = m_world->accessBody(bodyId);

    EXPECT_TRUE(bodyId.isValid());
    EXPECT_TRUE(body.getBodyId().isValid());
    EXPECT_EQ(bodyId, body.getBodyId());

    m_world->removeBody(bodyId);
}

TEST_F(WorldTest, iterate_bodies)
{

    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    for(int i = 0; i < 100; ++i)
    {
        m_world->createBody(bodyCInfo, true);
    }


    auto& bodyManager = m_world->getBodyManager();
    auto bodyIter = bodyManager.getBodyIter();
    while(bodyIter.isValid())
    {
        cvBodyId bodyId = *bodyIter;
        const cvBody& body = bodyManager.getBody(bodyId);
        EXPECT_EQ(bodyCInfo.m_shape, body.getShape());
        bodyIter++;
    }
}

TEST_F(WorldTest, integrate)
{
    cvBodyCInfo bodyCInfo;
    bodyCInfo.m_shape = std::shared_ptr<cvShape>(
            cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.05f));

    cvBodyId id = m_world->createBody(bodyCInfo, true);
    m_world->setBodyAngularVelocity(id, 1.0f);
    m_world->setBodyLinearVelocity(id, cvVec2f(1.0f, 2.0f));
    m_world->integrate(0.01f);
    cvTransform xform = m_world->getBody(id).getTransform();

    EXPECT_NEAR(0.01f, xform.m_Rotation, CV_FLOAT_EPS);
    EXPECT_EQ(cvVec2f(0.01f, 0.02f), xform.m_Translation);

    m_world->integrate(0.01f);
    xform = m_world->getBody(id).getTransform();
    EXPECT_NEAR(0.02f, xform.m_Rotation, CV_FLOAT_EPS);
    EXPECT_EQ(cvVec2f(0.02f, 0.04f), xform.m_Translation);
}
