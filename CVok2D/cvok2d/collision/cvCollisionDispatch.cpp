#include "cvCollisionDispatch.h"
#include "cvManifold.h"


cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];

void _colCirclevsCircle(const cvShape* shapeA, const cvShape* shapeB, cvMat33& matA,
        cvMat33& matB, cvManifoldPt& pt)
{
    
}

void _colCirclevsPoly(const cvShape* shapeA, const cvShape* shapeB, cvMat33& matA,
        cvMat33& matB, cvManifoldPt& pt)
{
}

void _colPolyvsPoly(const cvShape* shapeA, const cvShape* shapeB, cvMat33& matA,
        cvMat33& matb, cvManifoldPt& pt)
{
}


void _initCollisionDispatchFns()
{
    g_collisionFunction[cvShape::eCircle][cvShape::eCircle] = _colCirclevsCircle;
    g_collisionFunction[cvShape::eCircle][cvShape::ePolygon] = _colCirclevsPoly;
    g_collisionFunction[cvShape::ePolygon][cvShape::ePolygon] = _colPolyvsPoly;
}

