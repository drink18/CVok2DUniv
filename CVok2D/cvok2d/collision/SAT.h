#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"
#include "cvQuery.h"

class cvCircle;
class cvPolygonShape;

namespace  SAT
{
    struct SATResult
    {
        cvVec2f closetPt;
        cvVec2f normal;
        float distance;
        int ei0;
        int ei1;
        cvVec2f ep0;
        cvVec2f ep1;
        bool penetrated = false;
    };

    SATResult _circleToPolygon(const cvCircle& circle, const cvPolygonShape& poly,
            const cvMat33& transA, const cvMat33& transB);
}
