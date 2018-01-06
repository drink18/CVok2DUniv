#include "cvDistance.h"

#include <algorithm>
#include <iostream>

using namespace std;

namespace cvDist
{

    cvPt2LineClosestPt pointDistanceToLine(const cvVec2f& queryPt, const cvVec2f& p0, const cvVec2f& p1)
    {
        //find projected point
        cvVec2f d = p1 - p0;
        float lenD = d.length();
        d /= lenD;
        float u = 1.0f - (queryPt - p0).dot(d) / lenD;
        float v = 1.0f - u;

        cvPt2LineClosestPt res;
        u = min(1.0f, max(u, 0.0f));
        v = min(1.0f, max(v, 0.0f));
        res.pt = p0 * u + p1 * v;
        res.u = u;
        res.v = v;

        return res;
    }
}
