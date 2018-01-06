#pragma once

#include <core/cvMath.h>

class cvShape;
class cvConvexShape;
namespace GJK
{
    enum class GJKResult
    {
        Separated,
        Overlapping
    };

    struct GJKContext
    {
        const cvConvexShape& shapeA;
        const cvConvexShape& shapeB;
        cvMat33 poseA;
        cvMat33 poseB;
    };

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance);

    float pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);
}
