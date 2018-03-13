#pragma  once

#include <core/cvMath.h>
#include <shape/cvShape.h>

class cvShape;
class cvManifoldPt;

using cvCollisionFn = void(*)(const cvShape& shapeA, const cvShape& shapeB,
        const cvMat33& matA, const cvMat33& matB, cvManifoldPt& pt);

extern cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];




