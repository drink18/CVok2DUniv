#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>

TEST(cvCircle, Init)
{
    cvCircle circle(cvVec2f(0, 0), 2.0f);

    EXPECT_EQ(circle.getShapeType(), cvShape::eCircle);
}

TEST(cvCircle, getSupport)
{
    cvCircle circle(cvVec2f(0, 0), 2.0f);

    auto s = circle.getSupport(cvVec2f(1.0f, 0.0));
    cvVec2f support = s.p;
    EXPECT_NEAR(support.x, 2.0f, CV_FLOAT_EPS);
    EXPECT_NEAR(support.m_y, 0.0f, CV_FLOAT_EPS);

    s = circle.getSupport(cvVec2f(0.0f, 1.0f));
    support = s.p;
    EXPECT_NEAR(support.x, 0.0f, CV_FLOAT_EPS);
    EXPECT_NEAR(support.m_y, 2.0f, CV_FLOAT_EPS);
}


