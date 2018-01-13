#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"

class cvShape;
class cvConvexShape;
namespace GJK
{

    struct GJKResult 
    {
        cvVec2f closetPt;
        cvVec2f normal;
        float distance;
    };

    struct GJKContext
    {
        const cvConvexShape& shapeA;
        const cvConvexShape& shapeB;
        cvMat33 poseA;
        cvMat33 poseB;
    };

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance);

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);
}
