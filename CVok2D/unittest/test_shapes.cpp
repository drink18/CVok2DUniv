#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>

TEST(cvCircle, Init)
{
    cvCircle circle(cvVec2f(0, 0), 2.0f);

    EXPECT_EQ(circle.getShapeType(), cvShape::eCircle);
}

TEST(cvCircle,  GetCorrectAABB)
{
    cvCircle circle(cvVec2f(0.5f, 0.5f), 2.0f);

    EXPECT_EQ(circle.getShapeType(), cvShape::eCircle);
    const cvAabb& aabb = circle.getAabb();
    EXPECT_EQ(cvVec2f(-1.5f, -1.5f), aabb.m_Min);
    EXPECT_EQ(cvVec2f(2.5f, 2.5f), aabb.m_Max);
}

TEST(cvCircle, getSupport)
{
    cvCircle circle(cvVec2f(0, 0), 2.0f);

    auto s = circle.getSupport(cvVec2f(1.0f, 0.0));
    cvVec2f support = s.p;
    EXPECT_EQ(cvVec2f(2.0f, 0.0f), support);

    s = circle.getSupport(cvVec2f(0.0f, 1.0f));
    support = s.p;
    EXPECT_EQ(cvVec2f(0.0f, 2.0f), support);
}

TEST(cvPolygonShape,  GetCorrectAABB)
{
    auto box = cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), 0.01f);

    cvAabb aabb = box->getAabb();
    EXPECT_EQ(cvVec2f(-0.5f, -0.5f), aabb.m_Min);
}

