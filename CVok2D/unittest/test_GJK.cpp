#include <gtest/gtest.h>

#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>
#include <cvok2d/collision/cvDistance.h>
#include <cvok2d/collision/GJK.h>
#include <memory>

using namespace GJK;
class TestGJKTest : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        cvBox = unique_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(5.0f, 5.0f), cvVec2f(15.0f, 15.0f), .05f));
    }
    unique_ptr<cvPolygonShape> cvBox;
};

TEST_F(TestGJKTest, BasicCase)
{
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

TEST_F(TestGJKTest, Overlapping)
{
    cvVec2f q(7, 9);
    auto res = pointToConvex(q, *cvBox);
    EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
}

TEST_F(TestGJKTest, Overlapping_OnInteriorEdge)
{
    cvVec2f q(9, 9);
    auto res = pointToConvex(q, *cvBox);
    EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
}

TEST_F(TestGJKTest, Overlapping_VertexOverlap)
{
    {
        cvVec2f q(5.0f, 5.0f);
        auto res = pointToConvex(q, *cvBox);
        EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
    }

    {
        cvVec2f q(15.0f, 5.0f);
        auto res = pointToConvex(q, *cvBox);
        EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
    }

    {
        cvVec2f q(15.0f, 15.0f);
        auto res = pointToConvex(q, *cvBox);
        EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
    }

    {
        cvVec2f q(5.0f, 15.0f);
        auto res = pointToConvex(q, *cvBox);
        EXPECT_EQ(GJKResult::GJK_OVERLAP, res.result);
    }
}

TEST(TestGJK, pt2BoxEdge)
{
    cvVec2f q(0, 2);
    auto box = cvPolygonShape::createBox(cvVec2f(-1.0f, -1.0f), cvVec2f(1.0f, 1.0f), 0.01f);
    auto res = pointToConvex(q, *box);
    EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
}

TEST(TestGJK, MinkowskiDiff)
{
    auto ba = cvPolygonShape::createBox(cvVec2f(-5.0f, -5.0f), cvVec2f(5.0f, 5.0f), 0.05f);
    auto bb = cvPolygonShape::createBox(cvVec2f(11.5, -7.5f), cvVec2f(26.5f, 7.5f), 0.01f);
    cvMat33 ma; ma.setIdentity();
    cvMat33 mb; mb.setIdentity();
    cvShapeQueryInput input(*ba,*bb, ma, mb);
    
    auto left = _getSupportOnMinkowsiDiff(input, cvVec2f(-1, 0));
    EXPECT_NEAR(-31.5f, left.p.x, CV_FLOAT_EPS);

    auto right = _getSupportOnMinkowsiDiff(input, cvVec2f(1, 0));
    EXPECT_NEAR(-6.5f, right.p.x, CV_FLOAT_EPS);

    auto  up = _getSupportOnMinkowsiDiff(input, cvVec2f(0, 1));
    EXPECT_NEAR(12.5f, up.p.y, CV_FLOAT_EPS);

    auto  down = _getSupportOnMinkowsiDiff(input, cvVec2f(0, -1));
    EXPECT_NEAR(-12.5f, down.p.y, CV_FLOAT_EPS);
}

TEST(TestGJK, MinkowskiDiff_NonNormalizedD)
{
    auto ba = cvPolygonShape::createBox(cvVec2f(-5.0f, -5.0f), cvVec2f(5.0f, 5.0f), 0.05f);
    auto bb = cvPolygonShape::createBox(cvVec2f(11.5, -7.5f), cvVec2f(26.5f, 7.5f), 0.01f);
    cvMat33 ma; ma.setIdentity();
    cvMat33 mb; mb.setIdentity();
    cvShapeQueryInput input(*ba,*bb, ma, mb);
    
    auto left = _getSupportOnMinkowsiDiff(input, cvVec2f(-1, 0));
    EXPECT_NEAR(-31.5f, left.p.x, CV_FLOAT_EPS);

    auto right = _getSupportOnMinkowsiDiff(input, cvVec2f(25, -0));
    EXPECT_NEAR(-6.5f, right.p.x, CV_FLOAT_EPS);

    auto  up = _getSupportOnMinkowsiDiff(input, cvVec2f(0, 1));
    EXPECT_NEAR(12.5f, up.p.y, CV_FLOAT_EPS);

    auto  down = _getSupportOnMinkowsiDiff(input, cvVec2f(0, -1));
    EXPECT_NEAR(-12.5f, down.p.y, CV_FLOAT_EPS);
}

TEST(TestGJK, pt2Box)
{
    auto ba = cvPolygonShape::createBox(cvVec2f(-31.5f, -12.5f), cvVec2f(12.5f, -6.5f), 0.01f);
    cvVec2f q(0, 0);
    auto res = pointToConvex(q, *ba);

    EXPECT_EQ(GJKResult::GJK_GOOD, res.result);
}

TEST(TestGJK, ParallelBoxes)
{
    auto ba = cvPolygonShape::createBox(cvVec2f(5.0f, 5.0f), 0.05f);
    auto bb = cvPolygonShape::createBox(cvVec2f(7.5f, 7.5f),  0.01f);
    cvMat33 ma; ma.setIdentity();
    cvMat33 mb; mb.setTranslation(cvVec2f(19, 0));
    cvShapeQueryInput input(*ba,*bb, ma, mb);

    auto res = cvGJKConvexToConvex(input);
    EXPECT_EQ(true, res.m_succeed);
    EXPECT_NEAR(6.5f, res.m_distance, CV_FLOAT_EPS);
}