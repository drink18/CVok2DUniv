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

     cvPt2TriangleClosestPt pointDistanceToTriangle(const cvVec2f& q, const cvVec2f& a, const cvVec2f& b, const cvVec2f& c)
     {
         cvPt2TriangleClosestPt res;
        //first, barycentric coord with triangle edge line segs
        cvPt2LineClosestPt ab, bc, ca;
        ab = pointDistanceToLine(q, a, b);
        bc = pointDistanceToLine(q, b, c);
        ca = pointDistanceToLine(q, c, a);
        float uab = ab.u;
        float vab = ab.v;
        float ubc = bc.u;
        float vbc = bc.v;
        float uca = ca.u;
        float vca = ca.v;

        //barycentric coord of triangle
        float area = (b - a).cross(c -a);
        float uABC = (b - q).cross(c -q);
        float vABC = (c - q).cross(a -q);
        float wABC = (a - q).cross(b -q);
        res.u = uABC;
        res.v = vABC;
        res.w = wABC;

        //test vertex region
        if(uca < 0 && vab < 0 )
        {
            res.pt = a;
            return res;
        }
        else if(uab < 0 && vbc < 0)
        {
            res.pt = b;
            return res;
        }
        else if(ubc < 0 && vca < 0)
        {
            res.pt = c;
            return res;
        }

        // test edge region
        if(uab > 0 && vab > 0 && wABC <=0) //edge ab
        {
            res.pt = ab.pt;
            return res;
        }
        else if(ubc > 0 && vbc > 0 && uABC <=0) //edge bc
        {
            res.pt = bc.pt;
            return res;
        }
        else if(uca > 0 && vca > 0 && vABC <= 0) // edge ca
        {
            res.pt = ca.pt;
            return res;
        }

        //interior
        res.pt = q;
        return res;
     }
}
