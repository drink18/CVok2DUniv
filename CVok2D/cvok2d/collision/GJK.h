#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"
#include <memory>

class cvShape;
class cvConvexShape;

namespace GJK
{
    using namespace cvDist;
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

    struct  cvShapeQueryInput
    {
        cvShapeQueryInput(const cvConvexShape& _shapeA, const cvConvexShape& _shapeB,
                const cvMat33& _poseA, const cvMat33& _poseB)
            :shapeA(_shapeA), shapeB(_shapeB), poseA(_poseA), poseB(_poseB)
        {
        }
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

    struct cvConvex2ConvexGJKResult
    {
        cvVec2f m_pA;
        cvVec2f m_pB;
        float m_distance;
    };

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input);

    cvConvex2ConvexGJKResult cvGJKConvexToConvex(const cvShapeQueryInput& input);
}
