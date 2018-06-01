#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>
#include <cvok2d/core/cvAabb.h>

TEST(cvAabb, Init)
{
	cvAabb a1;
	cvAabb a2(cvVec2f(0, 0), cvVec2f(2, 1));
	
    EXPECT_EQ(cvVec2f(FLT_MAX, FLT_MAX), a1.m_Min);
    EXPECT_EQ(cvVec2f(FLT_MIN, FLT_MIN), a1.m_Max);

    EXPECT_EQ(cvVec2f(0, 0), a2.m_Min);
    EXPECT_EQ(cvVec2f(2, 1), a2.m_Max);
}

TEST(cvAabb, include)
{
	cvAabb a;
	cvAabb b(cvVec2f(0, 0), cvVec2f(1, 1));
	cvAabb b1(cvVec2f(-1, -1), cvVec2f(0, 0));

	a.include(b);
    EXPECT_EQ(cvVec2f(0, 0), a.m_Min);
    EXPECT_EQ(cvVec2f(1, 1), a.m_Max);

	a.include(b1);
    EXPECT_EQ(cvVec2f(-1, -1), a.m_Min);
    EXPECT_EQ(cvVec2f(1, 1), a.m_Max);
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

	EXPECT_TRUE(a1.overlaps(a2));
	EXPECT_TRUE(a1.overlaps(a3));
	EXPECT_TRUE(a1.overlaps(a4));
}

TEST(cvAabb, overlaps_self)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));

	EXPECT_TRUE(a1.overlaps(a1));
}

TEST(cvAabb, overlaps_x_identical)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));
	cvAabb a2(cvVec2f(0, -3), cvVec2f(2, 0));

	EXPECT_FALSE(a1.overlaps(a2));
}

TEST(cvAabb, overlaps_y_identical)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));
	cvAabb a2(cvVec2f(-2, 0), cvVec2f(0, 3));

	EXPECT_FALSE(a1.overlaps(a2));
}

TEST(cvAabb, expand)
{
	cvAabb a1(cvVec2f(0, 0), cvVec2f(2, 3));
    cvVec2f exp(1, 2);

    a1.expand(exp);
    EXPECT_EQ(cvVec2f(-1, -2), a1.m_Min);
    EXPECT_EQ(cvVec2f(3, 5), a1.m_Max);
}
