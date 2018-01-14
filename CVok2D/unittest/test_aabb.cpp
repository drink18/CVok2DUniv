#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>
#include <cvok2d/core/cvAabb.h>

TEST(cvAabb, Init)
{
	cvAabb a1;
	cvAabb a2(cvVec2f(0, 0), cvVec2f(2, 1));
	
	EXPECT_NEAR(a1.m_Min.x, FLT_MAX, CV_FLOAT_EPS);
	EXPECT_NEAR(a1.m_Min.m_y, FLT_MAX, CV_FLOAT_EPS);
	EXPECT_NEAR(a1.m_Max.x, FLT_MIN, CV_FLOAT_EPS);
	EXPECT_NEAR(a1.m_Max.m_y, FLT_MIN, CV_FLOAT_EPS);

	EXPECT_NEAR(a2.m_Min.x, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(a2.m_Min.m_y, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(a2.m_Max.x, 2, CV_FLOAT_EPS);
	EXPECT_NEAR(a2.m_Max.m_y, 1, CV_FLOAT_EPS);
}

TEST(cvAabb, include)
{
	cvAabb a;
	cvAabb b(cvVec2f(0, 0), cvVec2f(1, 1));
	cvAabb b1(cvVec2f(-1, -1), cvVec2f(0, 0));

	a.include(b);
	EXPECT_NEAR(a.m_Min.x, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Min.m_y, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Max.x, 1, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Max.m_y, 1, CV_FLOAT_EPS);

	a.include(b1);
	EXPECT_NEAR(a.m_Min.x, -1, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Min.m_y, -1, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Max.x, 1, CV_FLOAT_EPS);
	EXPECT_NEAR(a.m_Max.m_y, 1, CV_FLOAT_EPS);

}

TEST(cvAabb, contains)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));
	cvAabb a2(cvVec2f(0, 0), cvVec2f(1, 2));
	cvAabb a3(cvVec2f(0.1f, 0.1f), cvVec2f(4, 3));

	EXPECT_TRUE(a1.contains(a1));
	EXPECT_TRUE(a1.contains(a2));
	EXPECT_FALSE(a1.contains(a3));
}

TEST(cvAabb, overlaps)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));
	cvAabb a2(cvVec2f(0, 0), cvVec2f(1, 2));
	cvAabb a3(cvVec2f(0.1f, 0.1f), cvVec2f(4, 3));
	cvAabb a4(cvVec2f(-0.1f, -0.1f), cvVec2f(1, 2));

	EXPECT_TRUE(a1.overlaps(a1));
	EXPECT_TRUE(a1.overlaps(a2));
	EXPECT_TRUE(a1.overlaps(a3));
	EXPECT_TRUE(a1.overlaps(a4));
}
