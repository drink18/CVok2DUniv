#include <gtest/gtest.h>
#include <core/cvMath.h>

TEST(cvMat33, SetIdentity)
{
	cvMat33 m;
	m.setIdentity();


    EXPECT_EQ(cvVec3f(1.0f, 0.0f, 0.0f), m.m_cols[0]);
    EXPECT_EQ(cvVec3f(0.0f, 1.0f, 0.0f), m.m_cols[1]);
    EXPECT_EQ(cvVec3f(0.0f, 0.0f, 1.0f), m.m_cols[2]);
}

TEST(cvMat33, SetRotation)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);

    EXPECT_EQ(cvVec3f(0.0f, -1.0f, 0.0f), m.m_cols[0]);
    EXPECT_EQ(cvVec3f(1.0f, 0.0f, 0.0f), m.m_cols[1]);
}


TEST(cvMat33, RotateVecotr)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(90);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformVector(v, tv);
    EXPECT_EQ(cvVec2f(0.0f, 1.0f), tv);

	m.setRotationDeg(-90);
	m.transformVector(v, tv);
    EXPECT_EQ(cvVec2f(0.0f, -1.0f), tv);
}

TEST(cvMat33, RotateVecotr_45Deg)
{
	cvMat33 m;
	m.setIdentity();
	m.setRotationDeg(45);

	cvVec2f v(1, 0);
	cvVec2f tv;
	m.transformVector(v, tv);
    EXPECT_EQ(cvVec2f(0.7071067811f, 0.7071067811f), tv);

	m.setRotationDeg(-45);
	m.transformVector(v, tv);
    EXPECT_EQ(cvVec2f(0.7071067811f, -0.7071067811f), tv);
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

    EXPECT_EQ(cvVec2f(0.0f, 2.0f), tv);
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
    EXPECT_EQ(cvVec2f(0.0f, 2.0f), tv);

	cvMat33 mm;
	m2.mul(m1, mm);
	mm.transformPoint(v, tv);
    EXPECT_EQ(cvVec2f(-1.0f, 1.0f), tv);
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
    EXPECT_EQ(cvVec2f(0.0f, 2.0f), tv);

	cvMat33 mm(m2);
	mm.setMul(m1);
	mm.transformPoint(v, tv);
    EXPECT_EQ(cvVec2f(-1.0f, 1.0f), tv);
}

TEST(cvTransform, toMat33)
{
	cvMat33 m;
	cvTransform xform(cvVec2f(1.0f, 0.0f), DEG2RAD(90));
	xform.toMat33(m);

	cvVec2f v(1.0f, 0.0f);
	cvVec2f tv;
	m.transformPoint(v, tv);

    EXPECT_EQ(cvVec2f(1.0f, 1.0f), tv);
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
    EXPECT_EQ(cvVec2f(0.0f, 2.0f), tv);

	cvMat33 mm;
	m2.mul(m1, mm);
	mm.transformPoint(v, tv);
    EXPECT_EQ(cvVec2f(-1.0f, 1.0f), tv);
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
    EXPECT_EQ(cvVec2f(0.0f, 2.0f), tv);

	cvMat33 mm = m2;
    mm *= m1;
	mm.transformPoint(v, tv);
    EXPECT_EQ(cvVec2f(-1.0f, 1.0f), tv);
}

void testMatrixIdentity(const cvMat33& m1, const cvMat33& m2)
{
    EXPECT_EQ(m1.m_cols[0], m2.m_cols[0]);
    EXPECT_EQ(m1.m_cols[1], m2.m_cols[1]);
    EXPECT_EQ(m1.m_cols[2], m2.m_cols[2]);
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
