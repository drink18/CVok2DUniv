#pragma  once

#include <core/cvMath.h>
#include <shape/cvShape.h>

class cvShape;
class cvManifoldPoint;

using cvCollisionFn = void(*)(const cvShape& shapeA, const cvShape& shapeB,
        const cvMat33& matA, const cvMat33& matB, std::vector<cvManifoldPoint>& manifolds);

extern cvCollisionFn g_collisionFunction[cvShape::eShapeType_Count][cvShape::eShapeType_Count];




