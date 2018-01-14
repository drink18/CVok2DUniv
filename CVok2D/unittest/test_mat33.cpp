#include <gtest/gtest.h>
#include <core/cvMath.h>

TEST(cvMat33, SetIdentity)
{
	cvMat33 m;
	m.setIdentity();


	EXPECT_NEAR(m.m_cols[0].x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[0].y, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[0].z, 0.0f, CV_FLOAT_EPS);

	EXPECT_NEAR(m.m_cols[1].x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].y, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].z, 0.0f, CV_FLOAT_EPS);

	EXPECT_NEAR(m.m_cols[2].x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[2].y, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[2].z, 1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, SetRotation)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);

	EXPECT_NEAR(m.m_cols[0].x, 0.0f, CV_FLOAT_EPS );
	EXPECT_NEAR(m.m_cols[0].y, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(m.m_cols[1].y, 0.0f, CV_FLOAT_EPS);
}


TEST(cvMat33, RotateVecotr)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformVector(v, tv);
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);

	m.setRotationDeg(-90);
	m.transformVector(v, tv);
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, -1.0f, CV_FLOAT_EPS);
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

	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 2.0f, CV_FLOAT_EPS);
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
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm;
	m2.mul(m1, mm);
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);
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
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm(m2);
	mm.setMul(m1);
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);
}

TEST(cvTransform, toMat33)
{
	cvMat33 m;
	cvTransform xform(cvVec2f(1.0f, 0.0f), 90);
	xform.toMat33(m);

	cvVec2f v(1.0f, 0.0f);
	cvVec2f tv;
	m.transformPoint(v, tv);

	EXPECT_NEAR(tv.x, 1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, OpMul)
{
	cvMat33 m1;
	m1.setIdentity();
	m1.setRotationDeg(90);

	cvMat33 m2;
	m2.setIdentity();
	m2.setTranslation(cvVec2f(0, 1));

	cvMat33 m = m1 * m2;

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm;
	m2.mul(m1, mm);
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);
}

TEST(cvMat33, OpSelfMul)
{
	cvMat33 m1;
	m1.setIdentity();
	m1.setRotationDeg(90);

	cvMat33 m2;
	m2.setIdentity();
	m2.setTranslation(cvVec2f(0, 1));


    cvMat33 m = m1;
    m *= m2;

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, 0.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 2.0f, CV_FLOAT_EPS);

	cvMat33 mm = m2;
    mm *= m1;
	mm.transformPoint(v, tv);
	EXPECT_NEAR(tv.x, -1.0f, CV_FLOAT_EPS);
	EXPECT_NEAR(tv.y, 1.0f, CV_FLOAT_EPS);
}

void testMatrixIdentity(const cvMat33& m1, const cvMat33& m2)
{
    EXPECT_NEAR(m1.m_cols[0].x, m2.m_cols[0].x, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[0].y, m2.m_cols[0].y, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[0].z, m2.m_cols[0].z, CV_FLOAT_EPS);

    EXPECT_NEAR(m1.m_cols[1].x, m2.m_cols[1].x, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[1].y, m2.m_cols[1].y, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[1].z, m2.m_cols[1].z, CV_FLOAT_EPS);

    EXPECT_NEAR(m1.m_cols[2].x, m2.m_cols[2].x, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[2].y, m2.m_cols[2].y, CV_FLOAT_EPS);
    EXPECT_NEAR(m1.m_cols[2].z, m2.m_cols[2].z, CV_FLOAT_EPS);
}

TEST(cvMat33, invertIdentity)
{
    cvMat33 iden;
    cvMat33 invIden;
    iden.getInvert(invIden);

    testMatrixIdentity(iden, invIden);
}

TEST(cvMat33, invert)
{
    cvMat33 rotM;
    rotM.setRotationDeg(12);
    rotM.setTranslation(cvVec2f(12, -9));
    cvMat33 invRotM;
    rotM.getInvert(invRotM);

    cvMat33 shouldBeIden = rotM * invRotM;

    cvMat33 identity;

    testMatrixIdentity( identity, shouldBeIden);
}
