#include <gtest/gtest.h>
#include <core/cvMath.h>


TEST(cvVec3f, Init)
{
	cvVec3f v1(1.1f, -0.2f, 1.5);

	EXPECT_NEAR(v1.m_x, 1.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_y, -0.2f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_z, 1.5, CV_FLOAT_EPS);
}

TEST(cvVec3f, Add)
{
	cvVec3f v1(1.1f, 0, 21);
	cvVec3f v2(0, 0.5f, 2);
	cvVec3f v3(0.1f, 0.2f, 3);

	v1.add(v2);

	EXPECT_NEAR(v1.m_x, 1.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_y, 0.5f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_z, 23.f, CV_FLOAT_EPS);

	cvVec3f v4; v4.setAdd(v2, v3);
	EXPECT_NEAR(v4.m_x, 0.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v4.m_y, 0.7f, CV_FLOAT_EPS);
	EXPECT_NEAR(v4.m_z, 5, CV_FLOAT_EPS);
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
        EXPECT_NEAR(pv.m_x, -1.0f, CV_FLOAT_EPS);
        EXPECT_NEAR(pv.m_y, 0.0f, CV_FLOAT_EPS);
        EXPECT_NEAR(pv.m_z, 0.0f, CV_FLOAT_EPS);
    }

    {
        cvVec3f pv;
        pv = v.project(y);
        EXPECT_NEAR(pv.m_x, 0.0f, CV_FLOAT_EPS);
        EXPECT_NEAR(pv.m_y, 1.0f, CV_FLOAT_EPS);
        EXPECT_NEAR(pv.m_z, 0.0f, CV_FLOAT_EPS);
    }
}

TEST(cvVec3f, Cross)
{
	cvVec3f v1(1, 0, 0);
	cvVec3f v2(0, 1, 0);

    cvVec3f v = v1.cross(v2);
    EXPECT_NEAR(v.m_x, 0, CV_FLOAT_EPS);
    EXPECT_NEAR(v.m_y, 0, CV_FLOAT_EPS);
    EXPECT_NEAR(v.m_z, 1, CV_FLOAT_EPS);
}
