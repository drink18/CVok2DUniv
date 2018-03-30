#pragma  once

#include <core/cvMath.h>
#include <shape/cvShape.h>

class cvShape;
struct  cvShapeQueryInput
{
    cvShapeQueryInput(const cvShape& _shapeA, const cvShape& _shapeB,
            const cvMat33& _poseA, const cvMat33& _poseB)
        :shapeA(_shapeA), shapeB(_shapeB), poseA(_poseA), poseB(_poseB)
    {
    }
    const cvShape& shapeA;
    const cvShape& shapeB;
    cvMat33 poseA;
    cvMat33 poseB;
};

