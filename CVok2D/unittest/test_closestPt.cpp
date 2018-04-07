#include <gtest/gtest.h>

#include <cvok2d/core/cvMath.h>
#include <cvok2d/shape/cvCircle.h>
#include <cvok2d/shape/cvPolygonShape.h>
#include <cvok2d/collision/cvDistance.h>
#include <world/cvWorld.h>
#include <world/cvBody.h>
#include <cvok2d/collision/GJK.h>
#include <cvok2d/collision/SAT.h>
#include <collision/cvCollisionDispatch.h>
#include <cvok2dInit.h>

#include <memory>

using namespace GJK;
using namespace SAT;

class TestCP : public ::testing::Test
{
public:
    shared_ptr<cvPolygonShape> m_box;
    shared_ptr<cvCircle> m_circle;
    unique_ptr<cvWorld> m_world;

    cvBodyId m_body1;
    cvBodyId m_body2;
    cvTransform m_tBox;
    cvTransform m_tCircle;

    void SetUp() override
    {
        m_box = shared_ptr<cvPolygonShape>(cvPolygonShape::createBox(cvVec2f(1, 1), 0.05f));
        m_circle = make_shared<cvCircle>(cvVec2f(0,0), 1.0f);
        cvTransform& t1 = m_tBox;
        t1.m_Translation.set(0, 0);
        //t1.m_Rotation = DEG2RAD(50);

        cvTransform& t2 = m_tCircle;
        t2.m_Translation.set(2.5f, 0);//17, 17);
        //t2.m_Rotation = DEG2RAD(45);
        //
        //
        cv2DInit();
    }

    void TearDown() override
    {
    }
};


TEST_F(TestCP, TestCircle_Box_Sep)
{
    cvMat33 m1; m_tBox.toMat33(m1);
    cvMat33 m2; m_tCircle.toMat33(m2);

    cvCollisionFn fn = g_collisionFunction[cvShape::eCircle][cvShape::ePolygon];
    cvManifold manifold;

    //auto res = _circleToPolygon(*m_circle, *m_box, m2, m1);
    (*fn)(*m_circle, *m_box, m2, m1, manifold);

    ASSERT_EQ(1, manifold.m_numPt);
    ASSERT_EQ(cvVec2f(1.0f, 0), manifold.m_points[0].m_point);
    ASSERT_EQ(cvVec2f(1.0f, 0), manifold.m_points[0].m_normal);
    ASSERT_FLOAT_EQ(0.5f, manifold.m_points[0].m_distance);
}

TEST_F(TestCP, TestCircle_Box_Overlap)
{
    m_tCircle.m_Translation.set(1.9f, 0);
    cvMat33 m1; m_tBox.toMat33(m1);
    cvMat33 m2; m_tCircle.toMat33(m2);

    cvCollisionFn fn = g_collisionFunction[cvShape::eCircle][cvShape::ePolygon];
    cvManifold manifold;

    //auto res = _circleToPolygon(*m_circle, *m_box, m2, m1);
    (*fn)(*m_circle, *m_box, m2, m1, manifold);

    ASSERT_EQ(1, manifold.m_numPt);
    ASSERT_EQ(cvVec2f(1.0f, 0), manifold.m_points[0].m_point);
    ASSERT_EQ(cvVec2f(1.0f, 0), manifold.m_points[0].m_normal);
    ASSERT_FLOAT_EQ(-0.1f, manifold.m_points[0].m_distance);
}

