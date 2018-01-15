#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"
#include <memory>

class cvShape;
class cvConvexShape;

namespace GJK
{
    struct GJKResult
    {
        enum Result {
            GJK_GOOD,
            GJK_OVERLAP
        };

        Result result;
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

    struct cvPointQueryInput
    {
        cvVec2f q;
        std::shared_ptr<cvConvexShape> shape;
        cvTransform shapeXForm;
    };

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance);

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input);
}
