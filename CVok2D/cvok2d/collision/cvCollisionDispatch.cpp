#include "cvCollisionDispatch.h"
#include "cvManifold.h"

#include <collision/GJK.h>
#include <collision/SAT.h>
#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <collision/cvManifold.h>



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
    manifold.m_normal = d;
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
    pt.m_point = wldB + d * rb;
	//pt.m_feature.init(cvManifoldPtFeature::MF_Vertex, cvManifoldPtFeature::MF_Vertex, 0, 0);
}


void __colCirclevsPoly(const cvCircle& circleA, const cvPolygonShape& polyB, const cvMat33& matA,
        const cvMat33& matB, cvManifold& manifold)
{
    using namespace GJK;
    float r = circleA.getRadius();

    manifold.m_numPt = 1;

    auto wldCetner = matA * circleA.getCenter();
    cvPointQueryInput input(wldCetner, polyB, matB);
    GJKResult res = cvPointToConvexShape(input);
    cvManifoldPoint& pt = manifold.m_points[0];
    if(res.result == GJKResult::GJK_GOOD)
    {
        manifold.m_normal = res.normal;
        pt.m_distance = res.distance - r;
        pt.m_point = res.closetPt;
    }
    else
    {
        auto satRes = SAT::_circleToPolygon(circleA, polyB, matA, matB);

        manifold.m_normal = satRes.normal;
        pt.m_point = satRes.pts[0].point;
        pt.m_distance = satRes.pts[0].distance;
    }
}

void _colCirclevsPoly(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, cvManifold& manifold)
{

    bool invert = shapeA.getShapeType() != cvShape::eCircle;
    if(invert)
    {
        __colCirclevsPoly(static_cast<const cvCircle&>(shapeB), static_cast<const cvPolygonShape&>(shapeA),
                matB, matA, manifold);

        cvManifoldPoint& pt = manifold.m_points[0];
        pt.m_point = pt.m_point + manifold.m_normal * pt.m_distance;
        manifold.m_normal *= -1; 
    }
    else
    {
        __colCirclevsPoly(static_cast<const cvCircle&>(shapeA), static_cast<const cvPolygonShape&>(shapeB),
                matA, matB, manifold);
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

    manifold.m_numPt = 0;
    if(res.m_succeed)
    {
        manifold.m_numPt = 1;
        manifold.m_normal = res.m_seperation;
        cvManifoldPoint& pt = manifold.m_points[0];
        pt.m_point = res.m_pB;
        pt.m_distance = res.m_distance;
    }
    else
    {
        // need to run SAT or EPA
        auto satRes = SAT::_polyToPoly(polyA, polyB, matA, matB);
        manifold.m_numPt = satRes.numPt;
        manifold.m_normal = satRes.normal;
        for(int i = 0; i < manifold.m_numPt; ++i)
        {
            cvManifoldPoint& pt = manifold.m_points[i];
            pt.m_point = satRes.pts[i].point;
            pt.m_distance = satRes.pts[i].distance;
        }
    }
}


void _initCollisionDispatchFns()
{
}

cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count] =
{
    // circle
    {
        _colCirclevsCircle,
        _colCirclevsPoly
    },
    // poly
    {
        nullptr,
        _colPolyvsPoly
    }

};

cvCollisionFn cvGetCollisionFn(cvShape::ShapeType t1, cvShape::ShapeType t2)
{
    return (t1 > t2) ? g_collisionFunction[t2][t1] : g_collisionFunction[t1][t2];
}
