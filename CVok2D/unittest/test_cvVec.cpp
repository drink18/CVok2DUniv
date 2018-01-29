#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>

TEST(cvVec2f, Init)
{
    cvVec2f v1(1.1f, -0.2f);

    EXPECT_EQ(v1, cvVec2f(1.1f, -0.2f));
}

TEST(cvVec2f, Add)
{
    cvVec2f v1(1.1f, 0);
    cvVec2f v2(0, 0.5f);
    cvVec2f v3(0.1f, 0.2f);

    v1.add(v2);

    EXPECT_EQ(v1, cvVec2f(1.1f, 0.5f));

    cvVec2f v4; v4.setAdd(v2, v3);
    EXPECT_EQ(v4, cvVec2f(0.1f, 0.7f));
}

TEST(cvVec2f, Sub)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(0, 4);

    cvVec2f v; v.setSub(v1, v2);
    EXPECT_EQ(v, cvVec2f(3.0f, -4.0f));
}

TEST(cvVec2f, Length)
{
    cvVec2f v(3, -4);
    EXPECT_NEAR(v.length(), 5.0f, CV_FLOAT_EPS);
}

TEST(cvVec2f, Proj)
{
    cvVec2f v1(1, 0);
    cvVec2f v2(0, 1);
    cvVec2f v3(-1, 1);

    cvVec2f pv;
    pv = v1.project(v2);
    EXPECT_EQ(pv, cvVec2f(0, 0));

    pv = v3.project(v1);
    EXPECT_EQ(pv, cvVec2f(-1.0f, 0));
}

TEST(cvVec2f, Cross)
{
    cvVec2f v1(1, 0);
    cvVec2f v2(0, 1);

    EXPECT_NEAR(v1.cross(v2), 1, CV_FLOAT_EPS);
    EXPECT_NEAR(v1.cross(v1), 0, CV_FLOAT_EPS);
    EXPECT_NEAR(v1.cross(v2), -v2.cross(v1), CV_FLOAT_EPS);
}

TEST(cvVec2f, Distance)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(0, 4);

    EXPECT_NEAR(v1.distance(v2), 5.0f, CV_FLOAT_EPS);
    EXPECT_NEAR(v2.distance(v1), 5.0f, CV_FLOAT_EPS);
    EXPECT_NEAR(v1.distance(v1), 0.0f, CV_FLOAT_EPS);
}

TEST(cvVec2f, MinMax)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(0, 4);

    cvVec2f minV = cvVec2f::min2(v1, v2);
    cvVec2f maxV = cvVec2f::max2(v1, v2);

    EXPECT_EQ(minV, cvVec2f(0, 0));
    EXPECT_EQ(maxV, cvVec2f(3, 4));
}

TEST(cvVec2f, equal)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(3 + CV_FLOAT_EPS / 2, 0);
    cvVec2f v3(3, CV_FLOAT_EPS * 2);


    EXPECT_TRUE(cvVec2f::equal(v1, v1));
    EXPECT_TRUE(cvVec2f::equal(v1, v2));
    EXPECT_FALSE(cvVec2f::equal(v3, v1));
}

TEST(cvVec2f, less)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(0, 4);
    cvVec2f v3(5, 6);

    EXPECT_FALSE(cvVec2f::less(v1, v2));
    EXPECT_TRUE(cvVec2f::less(v1, v3));
    EXPECT_FALSE(cvVec2f::less(v3, v1));
}

TEST(cvVec2f, greater)
{
    cvVec2f v1(3, 0);
    cvVec2f v2(0, 4);
    cvVec2f v3(5, 6);

    EXPECT_FALSE(cvVec2f::greater(v1, v2));
    EXPECT_FALSE(cvVec2f::greater(v1, v3));
    EXPECT_TRUE(cvVec2f::greater(v3, v1));
}

TEST(cvVec2f, OpAdd)
{
    cvVec2f v1(1.1f, 0);
    cvVec2f v2(0, 0.5f);

    cvVec2f v12 = v1 + v2;
    EXPECT_EQ(v12, cvVec2f(1.1f, 0.5f));
}

TEST(cvVec2f, OpSub)
{
    cvVec2f v1(1.1f, 0);
    cvVec2f v2(0, 0.5f);

    cvVec2f v12 = v2 - v1;
    EXPECT_EQ(v12, cvVec2f(-1.1f, 0.5f));
}

TEST(cvVec2f, OpMul)
{
    cvVec2f v1(1.1f, 0);

    cvVec2f v12 = v1 * 2.0f;
    EXPECT_EQ(v12, cvVec2f(2.2f, 0));
}

TEST(cvVec2f, OpDiv)
{
    cvVec2f v1(1.1f, 0);

    cvVec2f v12 = v1 / 2.0f;
    EXPECT_EQ(v12, cvVec2f(0.55f, 0));
}

TEST(cvVec2f, OpCompound)
{
    cvVec2f v1(1.0f, 0);
    cvVec2f v2(1.0f, 2.0);

    cvVec2f v12 = (v1 + v2) / 2.0f;
    EXPECT_EQ(v12, cvVec2f(1.0f, 1.0f));
}

TEST(cvVec2f, OpSelfAdd)
{
    cvVec2f v1(1.0f, 0);
    cvVec2f v2(1.0f, 2.0);

    v1 += v2;
    EXPECT_EQ(v1, cvVec2f(2.0f, 2.0f));
}

TEST(cvVec2f, OpSelfSub)
{
    cvVec2f v1(1.0f, 0);
    cvVec2f v2(1.0f, 2.0);

    v1 -= v2;
    EXPECT_EQ(v1, cvVec2f(0.0f, -2.0f));
}

TEST(cvVec2f, OpSelfMul)
{
    cvVec2f v1(1.0f, 0);

    v1 *= 2;
    EXPECT_EQ(v1, cvVec2f(2.0f, 0.0f));
}

TEST(cvVec2f, OpSelfDiv)
{
    cvVec2f v1(1.0f, 0);

    v1 /= 2;
    EXPECT_EQ(v1, cvVec2f(0.5f, 0.0f));
}

TEST(cvVec2f, Negate)
{
    cvVec2f v(1.0f, 2.0f);
    cvVec2f v1 = -v;

    EXPECT_EQ(v1, cvVec2f(-1.0f, -2.0f));
}
