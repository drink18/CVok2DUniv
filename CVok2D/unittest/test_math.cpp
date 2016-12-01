// unittest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <gtest/gtest.h>
#include <cvok2d/core/cvMath.h>

TEST(cvVec2f, Init)
{
	cvVec2f v1(1.1f, -0.2f);

	EXPECT_NEAR(v1.m_x, 1.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_y, -0.2f, CV_FLOAT_EPS);
}

TEST(cvVec2f, Add)
{
	cvVec2f v1(1.1f, 0);
	cvVec2f v2(0, 0.5f);
	cvVec2f v3(0.1f, 0.2f);

	v1.add(v2);

	EXPECT_NEAR(v1.m_x, 1.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v1.m_y, 0.5f, CV_FLOAT_EPS);

	cvVec2f v4; v4.setAdd(v2, v3);
	EXPECT_NEAR(v4.m_x, 0.1f, CV_FLOAT_EPS);
	EXPECT_NEAR(v4.m_y, 0.7f, CV_FLOAT_EPS);
}

TEST(cvVec2f, Sub)
{
	cvVec2f v1(3, 0);
	cvVec2f v2(0, 4);

	cvVec2f v; v.setSub(v1, v2);
	EXPECT_NEAR(v.m_x, 3.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(v.m_y, -4.0f, CV_FLOAT_EPS);
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
	EXPECT_NEAR(pv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(pv.m_y, 0.0f, CV_FLOAT_EPS);

	pv = v3.project(v1);
	EXPECT_NEAR(pv.m_x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(pv.m_y, 0.0f, CV_FLOAT_EPS);
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

	EXPECT_NEAR(minV.m_x, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(minV.m_y, 0, CV_FLOAT_EPS);
	EXPECT_NEAR(maxV.m_x, 3, CV_FLOAT_EPS);
	EXPECT_NEAR(maxV.m_y, 4, CV_FLOAT_EPS);
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


TEST(cvMat33, SetIdentity)
{
	cvMat33 m;
	m.setIdentity();


	EXPECT_NEAR(m.m_cols[0].m_x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[0].m_y, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[0].m_z, 0.0f, CV_FLOAT_EPS);

	EXPECT_NEAR(m.m_cols[1].m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].m_y, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].m_z, 0.0f, CV_FLOAT_EPS);

	EXPECT_NEAR(m.m_cols[2].m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[2].m_y, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[2].m_z, 1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, SetRotation)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);
	

	EXPECT_NEAR(m.m_cols[0].m_x, 0.0f, CV_FLOAT_EPS );
	EXPECT_NEAR(m.m_cols[0].m_y, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].m_x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].m_y, 0.0f, CV_FLOAT_EPS);
}


TEST(cvMat33, RotateVecotr)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformVector(v, tv);
	EXPECT_NEAR(tv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 1.0f, CV_FLOAT_EPS);

	m.setRotationDeg(-90);
	m.transformVector(v, tv);
	EXPECT_NEAR(tv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, -1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, TransformPoint)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);
	m.setTranslation(cvVec2f(0, 1));

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformPoint(v, tv);

	EXPECT_NEAR(tv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 2.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, Mul)
{
	cvMat33 m1;
	m1.setIdentity();
	m1.setRotationDeg(90);

	cvMat33 m2;
	m2.setIdentity();
	m2.setTranslation(cvVec2f(0, 1));

	cvMat33 m;
	m1.mul(m2, m);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformPoint(v, tv);
	EXPECT_NEAR(tv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm;
	m2.mul(m1, mm);
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.m_x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, SetMul)
{
	cvMat33 m1;
	m1.setIdentity();
	m1.setRotationDeg(90);

	cvMat33 m2;
	m2.setIdentity();
	m2.setTranslation(cvVec2f(0, 1));

	cvMat33 m(m1);
	m.setMul(m2);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformPoint(v, tv);
	EXPECT_NEAR(tv.m_x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm(m2);
	mm.setMul(m1);
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.m_x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 1.0f, CV_FLOAT_EPS);
}

TEST(cvTransform, toMat33)
{
	cvMat33 m;
	cvTransform xform(cvVec2f(1.0f, 0.0f), 90);
	xform.toMat33(m);

	cvVec2f v(1.0f, 0.0f);
	cvVec2f tv;
	m.transformPoint(v, tv);

	EXPECT_NEAR(tv.m_x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.m_y, 1.0f, CV_FLOAT_EPS);
}
