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
            float p =  na.dot(verts[i] - pt);
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

        int nEdge = (int)wldVerts.size();
        float maxD;
        int maxDEdgeIdx = 0;
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

    float _getPointPeneOnAxis(const cvVec2f& pt, const cvVec2f& axis, const cvVec2f& ptOnAxis,
            const cvConvexShape& cvxShape)
    {
        // we want to check if pt is inside shape projected on a given separating axis
        cvVec2f s = cvxShape.getSupport(axis).p;
        float pp = axis.dot(pt - ptOnAxis);
        float sp = axis.dot(s - ptOnAxis);

        return pp - sp;
    }

    SATResult _pointToPolygon(const cvVec2f& pt, const cvPolygonShape& poly, const cvMat33& trans)
    {
        SATResult res;

        cvMat33 invT;  trans.getInvert(invT);
        cvVec2f ptL = invT * pt; //point in local space of polygon

        bool sep = false;
        auto& verts = poly.getVertices();
        int nedge = (int)verts.size();
        cvVec2f e = (verts[1] - verts[0]).getNormalized();
        cvVec2f midEdge = (verts[1] + verts[0]) / 2; 
        cvVec3f sap3(e.x, e.y, 1);
        sap3 = sap3.cross(cvVec3f(0, 0, 1));
        cvVec2f sap2(sap3.x, sap3.y);

        float deepestPen = _getPointPeneOnAxis(ptL, sap2, midEdge, poly);
        int deepestPenEdge = 0;
        cvVec2f deepestAxis = sap2;

        if (deepestPen == 0.0f)
        {
            res.penetrated = false;
            return res;
        }

        for (int i = 1; i < nedge; ++i)
        {
            int nei = (i == nedge - 1) ? 0 : i + 1;
            e = (verts[nei] - verts[i]).getNormalized();
            midEdge = (verts[nei] + verts[i]) / 2;
            sap3.set(cvVec3f(e.x, e.y, 1));
            sap3 = sap3.cross(cvVec3f(0, 0, 1));
            sap2.set(sap3.x, sap3.y);

            float pen = _getPointPeneOnAxis(ptL, sap2, midEdge, poly);
            if (pen > deepestPen)
            {
                deepestPen = pen;
                deepestPenEdge = i;
                deepestAxis = sap2;
            }

            if(pen > 0)
            {
                res.penetrated = false;
                return res;
            }
        }

        // if we got here point is inside polygon
        res.penetrated = true;
        cvVec2f ptOnPoly = ptL - verts[deepestPenEdge];
        res.closetPt = pt - deepestAxis * deepestPen;
        res.distance = deepestPen;
        res.normal = deepestAxis;
        res.ei0 = deepestPenEdge;
        res.ei1 = (deepestPenEdge == nedge - 1) ? 0 : deepestPenEdge + 1;
        res.ep0 = verts[res.ei0];
        res.ep1 = verts[res.ei1];
        res.closetPt = trans * res.closetPt;
        res.ep0 = trans * res.ep0;
        res.ep1 = trans * res.ep1;
        trans.transformVector(res.normal);

        return res;
    }
}
