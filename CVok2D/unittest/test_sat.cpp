#include <gtest/gtest.h>

#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>
#include <cvok2d/collision/cvDistance.h>
#include <cvok2d/collision/SAT.h>
#include <memory>


using namespace SAT;
using namespace std;

class TestSAT : public ::testing::Test
{
public:
    virtual void SetUp()
    { 
        cvBox.reset(cvPolygonShape::createBox(cvVec2f(0.5f, 0.5f), .05f));
    }

    virtual void TearDown()
    {}


    unique_ptr<cvPolygonShape> cvBox;
};

TEST_F(TestSAT, pt2Box_sep)
{
    cvMat33 iden; iden.setIdentity();
    cvVec2f pt(1.25, 1.25);
    auto res = _pointToPolygon(pt, *cvBox, iden);
    EXPECT_FALSE(res.penetrated);
}

TEST_F(TestSAT, pt2Box_in)
{
    cvMat33 iden; iden.setIdentity();
    cvVec2f pt(0.26f, 0.25f);
    auto res = _pointToPolygon(pt, *cvBox, iden);
    EXPECT_TRUE(res.penetrated);
    EXPECT_EQ(cvVec2f(0.5f, 0.25f), res.closetPt);
    EXPECT_EQ(cvVec2f(1.0f, 0), res.normal);

    pt.set(0.25f, 0.26f);
    res = _pointToPolygon(pt, *cvBox, iden);
    EXPECT_TRUE(res.penetrated);
    EXPECT_EQ(cvVec2f(0.25f, 0.5f), res.closetPt);
    EXPECT_EQ(cvVec2f(0, 1.0f), res.normal);

    pt.set(0.25f, -0.26f);
    res = _pointToPolygon(pt, *cvBox, iden);
    EXPECT_TRUE(res.penetrated);
    EXPECT_EQ(cvVec2f(0.25f, -0.5f), res.closetPt);
    EXPECT_EQ(cvVec2f(0, -1.0f), res.normal);

    pt.set(-0.26f, 0.25f);
    res = _pointToPolygon(pt, *cvBox, iden);
    EXPECT_TRUE(res.penetrated);
    EXPECT_EQ(cvVec2f(-0.5f, 0.25f), res.closetPt);
    EXPECT_EQ(cvVec2f(-1.0f, 0), res.normal);
}
