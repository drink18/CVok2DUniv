#include "SAT.h"

#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <cmath>
#include <algorithm>

namespace SAT
{
    using namespace std;
    void _projectCvxOnAxis(const vector<cvVec2f> verts, const cvVec2f& axis, 
            const cvVec2f& pt, float& minP, float& maxP)
    {
        cvVec2f na = axis.getNormalized();

        float pv = na.dot(verts[0] - pt);
        minP = maxP =  na.dot(verts[0] - pt);

        for(int i = 1; i < verts.size(); ++i)
        {
            float p =  na.dot(verts[0] - pt);
            minP = std::min(minP, p);
            maxP = std::max(maxP, p);
        }
    }

    float _getPeneDistance(vector<cvVec2f> vertsWldA, vector<cvVec2f> vertsWldB,
            const cvVec2f& axis, const cvVec2f& pt)
    {
        float minA, maxA;
        float minB, maxB;

        _projectCvxOnAxis(vertsWldA, axis, pt, minA, maxA);
        _projectCvxOnAxis(vertsWldB, axis, pt, minB, maxB);

        if(maxA <= minB) // disjoint, a on left
        {
            return minB - maxA;
        }
        else if(maxB <= minA) //disjoint, a on right
        {
            return minA - maxB;
        }
        else //overlapping
        {
            if(minA > minB && maxA < minB) // a inside b
            {
                return -max(maxA - minB, maxB - minA);
            }
            else if(minB > minA && maxB < minA) // b inside a
            {
                return -max(maxB - minA, maxA - minB);
            }
            else
            {
            if(maxA > minB) // a on left
            {
                return minB - maxA;
            }
            else if(maxB > minA) // A on right
            {
                return minA - maxB;
            }
            }
        }

        cvAssertMsg(false, "should not get here");
        return 0;
    }

    SATResult _circleToPolygon(const cvCircle& circle, const cvPolygonShape& poly,
            const cvMat33& transA, const cvMat33& transB)
    {
        SATResult res;

        vector<cvVec2f> cVerts{transA * circle.getCenter()};

        vector<cvVec2f> wldVerts;
        wldVerts.reserve(poly.getVertices().size());
        for(auto& v : poly.getVertices())
        {
            cvVec2f wldV = transB * v;
            wldVerts.push_back(wldV);
        }

        int nEdge =wldVerts.size();
        float maxD;
        float maxDEdgeIdx = 0;
        cvVec2f edgeN = wldVerts[1] - wldVerts[0];
        edgeN.normalize();
        maxD = _getPeneDistance(cVerts, wldVerts, edgeN, wldVerts[0]);

        if(maxD > 0)
        {
            res.penetrated = false;
            return res;
        }

        for(int i = 1; i < nEdge; ++i)
        {
            int nei = i + 1; // next edge vertex index
            if(i == nEdge - 1)
            {
                nei = 0;
            }
            edgeN = wldVerts[nei] - wldVerts[i];
            edgeN.normalize();

            float p = _getPeneDistance(cVerts, wldVerts, edgeN, wldVerts[i]);

            if(p > 0)
            {
                res.penetrated = false;
                return res;
            }

            if(p < maxD)
            {
                maxD = p;
                maxDEdgeIdx = i;
            }
        }
        // project to deepest penetrated edge
        cvVec2f pt = wldVerts[maxDEdgeIdx];
        int nei = (maxDEdgeIdx == nEdge -1) ? 0 :(maxDEdgeIdx + 1);
        edgeN = wldVerts[nei] - pt;
        edgeN.normalize();

        res.closetPt = pt +  edgeN * edgeN.dot(cVerts[0] - pt) ;
        res.distance = maxD;

        cvVec3f en3(edgeN.x, edgeN.y, 1);
        cvVec3f n3 = en3.cross(cvVec3f(0, 0, 1));
        res.normal.set(n3.x, n3.y);
        res.ep0 = pt;
        res.ep1 = wldVerts[nei];
        res.ei0 = maxDEdgeIdx;
        res.ei1 = nei;

        return res;
    }
}
