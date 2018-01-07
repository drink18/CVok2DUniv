#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>
#include <cvok2d/collision/cvDistance.h>

#include <iostream>
using namespace std;

TEST(TestDistance, closestPt_Pt2Line)
{
    cvVec2f p0(-1.0f, 0);
    cvVec2f p1(1.0f, 0);
    cvVec2f q(0, 1.0f);


    auto res = cvDist::pointDistanceToLine(q, p0, p1);
    EXPECT_NEAR(0, res.pt.m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(0, res.pt.m_y, CV_FLOAT_EPS);
    EXPECT_NEAR(0.5f, res.u, CV_FLOAT_EPS);
    EXPECT_NEAR(0.5f, res.v, CV_FLOAT_EPS);
}

TEST(TestDistance, closestPt_Zone0)
{
    cvVec2f p0(-1.0f, 0);
    cvVec2f p1(1.0f, 0);
    cvVec2f q(-2.0f, 1.0f);


    auto res = cvDist::pointDistanceToLine(q, p0, p1);
    EXPECT_NEAR(-1.0f, res.pt.m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(0, res.pt.m_y, CV_FLOAT_EPS);
}

TEST(TestDistance, closestPt_Zone1)
{
    cvVec2f p0(-1.0f, 0);
    cvVec2f p1(1.0f, 0);
    cvVec2f q(2.0f, 1.0f);


    auto res = cvDist::pointDistanceToLine(q, p0, p1);
    EXPECT_NEAR(1.0f, res.pt.m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(0, res.pt.m_y, CV_FLOAT_EPS);
}

TEST(TestDistance, closestPt_Zone2)
{
    cvVec2f p0(-1.0f, 0);
    cvVec2f p1(1.0f, 0);
    cvVec2f q(2.0f, -1.0f);


    auto res = cvDist::pointDistanceToLine(q, p0, p1);
    EXPECT_NEAR(1.0f, res.pt.m_x, CV_FLOAT_EPS);
    EXPECT_NEAR(0, res.pt.m_y, CV_FLOAT_EPS);
}
