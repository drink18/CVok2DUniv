#include "cvCollisionDispatch.h"
#include "cvManifold.h"

#include <collision/GJK.h>
#include <collision/SAT.h>
#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <collision/cvManifold.h>


cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];

void _colCirclevsCircle(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, cvManifold& manifold)
{
    auto& circleA = static_cast<const cvCircle&>(shapeA);
    auto& circleB = static_cast<const cvCircle&>(shapeB);

    cvVec2f wldA = matA * circleA.getCenter();
    cvVec2f wldB = matB * circleB.getCenter();
    float ra = circleA.getRadius();
    float rb = circleB.getRadius();

    cvVec2f d = wldA - wldB;
    float len = d.length();

    manifold.m_numPt = 1;
    cvManifoldPoint& pt = manifold.m_points[0];

    if(len > CV_FLOAT_EPS)
    {
        d /= len;
    }
    else
    {
        d = cvVec2f(1.0f, 0);
    }

    pt.m_distance = len - ra - rb;
    pt.m_normal = d;
    pt.m_point = wldB + d * rb;

}

void _colCirclevsPoly(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, cvManifold& manifold)
{
    using namespace GJK;
    auto& circleA = static_cast<const cvCircle&>(shapeA);
    auto& polyB = static_cast<const cvPolygonShape&>(shapeB);
    cvVec2f c = circleA.getCenter();
    float r = circleA.getRadius();

    manifold.m_numPt = 1;
    cvManifoldPoint& pt = manifold.m_points[0];

    cvPointQueryInput input(matA * circleA.getCenter(), polyB, matB);
    GJKResult res = cvPointToConvexShape(input);
    if(res.result == GJKResult::GJK_GOOD)
    {
        pt.m_normal = res.normal;
        pt.m_point = res.closetPt;
        pt.m_distance = res.distance - r;
    }
    else
    {
        auto satRes = SAT::_circleToPolygon(circleA, polyB, matA, matB);
        pt.m_normal = satRes.normal;
        pt.m_point = satRes.closetPt;
        pt.m_distance = satRes.distance;
    }
}

void _colPolyvsPoly(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, cvManifold& manifold)
{
    using namespace GJK;
    auto& polyA = static_cast<const cvPolygonShape&>(shapeA);
    auto& polyB = static_cast<const cvPolygonShape&>(shapeB);
    cvShapeQueryInput input(polyA, polyB, matA, matB);
    auto res = cvGJKConvexToConvex(input);

    if(res.m_succeed)
    {
        manifold.m_numPt = 1;
        cvManifoldPoint& pt = manifold.m_points[0];
        pt.m_point = res.m_pA;
        pt.m_distance = res.m_distance;
        pt.m_normal = res.m_seperation;
    }
    else
    {
        // need to run SAT or EPA
    }
}


void _initCollisionDispatchFns()
{
    g_collisionFunction[cvShape::eCircle][cvShape::eCircle] = _colCirclevsCircle;
    g_collisionFunction[cvShape::eCircle][cvShape::ePolygon] = _colCirclevsPoly;
    g_collisionFunction[cvShape::ePolygon][cvShape::ePolygon] = _colPolyvsPoly;
}

