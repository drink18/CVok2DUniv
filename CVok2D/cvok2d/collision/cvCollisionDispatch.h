#pragma  once

#include <core/cvMath.h>
#include <shape/cvShape.h>

struct cvManifold;
struct cvManifoldPoint;
class cvShape;

using cvCollisionFn =void (*)(const cvShape& shapeA, const cvShape& shapeB,
        const cvMat33& matA, const cvMat33& matB, cvManifold& manifold);

extern cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];

cvCollisionFn cvGetCollisionFn(cvShape::ShapeType t1, cvShape::ShapeType t2);




