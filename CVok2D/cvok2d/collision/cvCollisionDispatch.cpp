#include "cvCollisionDispatch.h"
#include "cvManifold.h"

#include <collision/GJK.h>
#include <shape/cvShape.h>
#include <shape/cvPolygonShape.h>
#include <collision/cvManifold.h>


cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];

void _colCirclevsCircle(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, vector<cvManifoldPoint>& manifolds)
{
}

void _colCirclevsPoly(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, vector<cvManifoldPoint>& manifoldsc)
{
}

void _colPolyvsPoly(const cvShape& shapeA, const cvShape& shapeB, const cvMat33& matA,
        const cvMat33& matB, vector<cvManifoldPoint>& manifolds)
{
    using namespace GJK;
    auto& polyA = static_cast<const cvPolygonShape&>(shapeA);
    auto& polyB = static_cast<const cvPolygonShape&>(shapeB);
    cvShapeQueryInput input(polyA, polyB, matA, matB);
    auto res = cvGJKConvexToConvex(input);


    cvManifoldPoint pt;
    pt.m_point = res.m_pA;
    pt.m_distance = res.m_distance;
    manifolds.push_back(pt);

}


void _initCollisionDispatchFns()
{
    g_collisionFunction[cvShape::eCircle][cvShape::eCircle] = _colCirclevsCircle;
    g_collisionFunction[cvShape::eCircle][cvShape::ePolygon] = _colCirclevsPoly;
    g_collisionFunction[cvShape::ePolygon][cvShape::ePolygon] = _colPolyvsPoly;
}

