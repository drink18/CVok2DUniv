#pragma once

#include <core/cvMath.h>

namespace cvDist
{
    struct cvPt2LineClosestPt
    {
        cvVec2f pt; //closest point on line;
        float u; //barycenter coord
        float v; //barycenter coord
    };

     cvPt2LineClosestPt pointDistanceToLine(const cvVec2f& queryPt, const cvVec2f& p0, const cvVec2f& p1);
}
