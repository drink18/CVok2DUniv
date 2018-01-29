#include <gtest/gtest.h>
#include <core/cvMath.h>


TEST(cvVec3f, Init)
{
    cvVec3f v1(1.1f, -0.2f, 1.5);

    EXPECT_EQ(cvVec3f(1.1f, -0.2f, 1.5f), v1);
}

TEST(cvVec3f, Add)
{
    cvVec3f v1(1.1f, 0, 21);
    cvVec3f v2(0, 0.5f, 2);
    cvVec3f v3(0.1f, 0.2f, 3);

    v1.add(v2);

    EXPECT_EQ(cvVec3f(1.1f, 0.5f, 23.f), v1);

    cvVec3f v4; v4.setAdd(v2, v3);
    EXPECT_EQ(cvVec3f(0.1f, 0.7f, 5.0f), v4);
}

TEST(cvVec3f, Proj)
{
    cvVec3f x(1, 0, 0);
    cvVec3f y(0, 1, 0);
    cvVec3f z(0, 0, 1);

    cvVec3f v(-1, 1, 1);

    {
        cvVec3f pv;
        pv = v.project(x);
        EXPECT_EQ(cvVec3f(-1.0f, 0.0f, 0.0f), pv);
    }

    {
        cvVec3f pv;
        pv = v.project(y);
        EXPECT_EQ(cvVec3f(0.0f, 1.0f, 0.0f), pv);
    }
}

TEST(cvVec3f, Cross)
{
    cvVec3f v1(1, 0, 0);
    cvVec3f v2(0, 1, 0);

    cvVec3f v = v1.cross(v2);
    EXPECT_EQ(cvVec3f(0.0f, 0.0f, 1.0f), v);
}
