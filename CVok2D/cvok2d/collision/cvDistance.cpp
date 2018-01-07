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
        float v =  (queryPt - p0).dot(d) / lenD;
        float u = 1.0f - v;

        cvPt2LineClosestPt res;
        if(u < 0)
            res.pt = p1;
        else if(v < 0)
            res.pt = p0;
        else
            res.pt = p0 * u + p1 * v;

        res.u = u;
        res.v = v;

        return res;
    }
}
