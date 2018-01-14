#include <gtest/gtest.h>

#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>
#include <cvok2d/collision/cvDistance.h>
#include <cvok2d/collision/GJK.h>
#include <memory>

using namespace GJK;
TEST(TestGJK, BasicCase)
{
    auto cvBox = unique_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(5.0f, 5.0f), cvVec2f(15.0f, 15.0f), .05f));
    {
        cvVec2f q(0, 0);
        auto res = pointToConvex(q, *cvBox);

        EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
        EXPECT_NEAR(5.0f, res.closetPt.x, CV_FLOAT_EPS);
        EXPECT_NEAR(5.0f, res.closetPt.y, CV_FLOAT_EPS);
        EXPECT_NEAR(7.07106781f, res.distance, CV_FLOAT_EPS);
    }

    {
        cvVec2f q(0, 7.2);
        auto res = pointToConvex(q, *cvBox);

        EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
        EXPECT_NEAR(5.0f, res.closetPt.x, CV_FLOAT_EPS);
        EXPECT_NEAR(7.2f, res.closetPt.y, CV_FLOAT_EPS);
        EXPECT_NEAR(5.0f, res.distance, CV_FLOAT_EPS);
    }

    {
        cvVec2f q(18.0f, 7.2);
        auto res = pointToConvex(q, *cvBox);

        EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
        EXPECT_NEAR(15.0f, res.closetPt.x, CV_FLOAT_EPS);
        EXPECT_NEAR(7.2f, res.closetPt.y, CV_FLOAT_EPS);
        EXPECT_NEAR(3.0f, res.distance, CV_FLOAT_EPS);
    }

    {
        cvVec2f q(18.0f, 18.0f);
        auto res = pointToConvex(q, *cvBox);

        EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
        EXPECT_NEAR(15.0f, res.closetPt.x, CV_FLOAT_EPS);
        EXPECT_NEAR(15.0f, res.closetPt.y, CV_FLOAT_EPS);
        EXPECT_NEAR(4.24264050f, res.distance, CV_FLOAT_EPS);
    }
}
