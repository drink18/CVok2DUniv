#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"
#include "cvQuery.h"

class cvCircle;
class cvPolygonShape;
class cvConvexShape;

namespace  SAT
{
    struct SATResult
    {
        int numPt = 0;
        cvVec2f point[2];
        float distance[2];
        cvVec2f normal;
        int ei0;
        int ei1;
        cvVec2f ep0;
        cvVec2f ep1;
        bool penetrated = false;
    };

    SATResult _circleToPolygon(const cvCircle& circle, const cvPolygonShape& poly,
            const cvMat33& transA, const cvMat33& transB);

    SATResult _pointToPolygon(const cvVec2f& pt, const cvPolygonShape& poly, const cvMat33& trans);

    SATResult _polyToPoly(const cvPolygonShape& shapeA, const cvPolygonShape& shapeB,
            const cvMat33& matA, const cvMat33& matB);
}