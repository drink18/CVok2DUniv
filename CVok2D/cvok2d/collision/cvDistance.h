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

    struct cvPt2TriangleClosestPt
    {
        cvVec2f pt; //closest point on line;
        float u; //barycenter coord
        float v; //barycenter coord
        float w; //barycenter coord
    };

     cvPt2LineClosestPt pointDistanceToLine(const cvVec2f& queryPt, const cvVec2f& p0, const cvVec2f& p1);

     cvPt2TriangleClosestPt pointDistanceToTriangle(const cvVec2f& q, const cvVec2f& a, const cvVec2f& b, const cvVec2f& c);
}
