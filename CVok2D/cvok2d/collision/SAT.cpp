#include "SAT.h"

#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>
#include <cmath>
#include <algorithm>

namespace SAT
{
    using namespace std;

    int _prevEdge(int n, int edgeCnt)
    {
        if(n == 0)
        {
            return edgeCnt - 1;
        }

        return n - 1;
    }

    int _nextEdge(int n, int edgeCnt)
    {
        return (n + 1) % edgeCnt;
    }


    void _projectCvxOnAxis(const vector<cvVec2f> verts, const cvVec2f& axis, 
            const cvVec2f& pt, float& minP, float& maxP)
    {
        cvVec2f na = axis.getNormalized();

        minP = maxP =  na.dot(verts[0] - pt);

        for(int i = 1; i < verts.size(); ++i)
        {
            float p =  na.dot(verts[i] - pt);
            minP = std::min(minP, p);
            maxP = std::max(maxP, p);
        }
    }

    float _getPointPeneOnAxis(const cvVec2f& pt, const cvVec2f& axis, const cvVec2f& ptOnAxis,
            const cvConvexShape& cvxShape)
    {
        // we want to check if pt is inside shape(segement) projected on a given separating axis
        cvVec2f s = cvxShape.getSupport(axis).p;
        float pp = axis.dot(pt - ptOnAxis);
        float sp = axis.dot(s - ptOnAxis);

        return pp - sp;
    }

    float _getPeneDist(const cvVec2f& axis, const cvVec2f& pt, const cvConvexShape& cvxShape)
    {
        cvVec2f s = cvxShape.getSupport(-axis).p;
        float d = axis.dot(s - pt);
        return d;
    }

    SATResult _circleToPolygon(const cvCircle& circle, const cvPolygonShape& poly,
            const cvMat33& transA, const cvMat33& transB)
    {
        cvVec2f pt = transA * circle.getCenter();
        auto ptRes = _pointToPolygon(pt, poly, transB);

        SATResult res;
        res.numPt = 1;
        res.point[0] = ptRes.point[0];
        res.distance[0] = ptRes.distance[0] - circle.getRadius();
        res.normal = ptRes.normal;
        return res;
    }


    SATResult _pointToPolygon(const cvVec2f& pt, const cvPolygonShape& poly, const cvMat33& trans)
    {
        SATResult res;

        cvMat33 invT;  trans.getInvert(invT);
        cvVec2f ptL = invT * pt; //point in local space of polygon

        auto& verts = poly.getVertices();
        int nedge = (int)verts.size();
        cvVec2f e = (verts[1] - verts[0]).getNormalized();
        cvVec2f midEdge = (verts[1] + verts[0]) / 2; 
        cvVec2f sap2 = e.computePerpendicular();

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
            int nei = _nextEdge(i, nedge);
            e = (verts[nei] - verts[i]).getNormalized();
            midEdge = (verts[nei] + verts[i]) / 2;
            cvVec2f sap2 = e.computePerpendicular();

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
        res.numPt = 1;
        res.point[0] = ptL - deepestAxis * deepestPen;
        res.distance[0] = deepestPen;
        res.normal = deepestAxis;
        res.ei0 = deepestPenEdge;
        res.ei1 = (deepestPenEdge == nedge - 1) ? 0 : deepestPenEdge + 1;
        res.ep0 = verts[res.ei0];
        res.ep1 = verts[res.ei1];
        res.point[0] = trans * res.point[0];
        res.ep0 = trans * res.ep0;
        res.ep1 = trans * res.ep1;
        trans.transformVector(res.normal);

        return res;
    }

    void _clipEdgeWithNormal(cvVec2f& ep0, cvVec2f& ep1, const cvVec2f& planNormal,
            const cvVec2f& p)
    {
        float d0 = planNormal.dot(ep0 - p);
        float d1 = planNormal.dot(ep1 - p);

        if(d0 * d1 < -CV_FLOAT_EPS)
        {
            // clipping
            float dp = d0 > 0 ? d0 : d1;
            float dn = d0 > 0 ? d1 : d0;

            float t = - dn / (dp - dn);
            cvVec2f en = ep1 - ep0;

            if(d0 > 0)
            {
                ep0 = ep1 - en * t;
            }
            else
            {
                ep1 = ep0 + en * t;
            }
        }
    }

    float _getMinPenD(const cvPolygonShape& refShape, const cvPolygonShape& shape,
            const cvMat33& transA, const cvMat33& transB, int& minEdgeIndx, cvVec2f& enormal)
    {
        float smallestPen = -FLT_MAX;

        cvMat33 invB; transB.getInvert(invB);
        cvMat33 a2b = transA * invB;
        auto& verts = refShape.getVertices();
        int edgeCount = refShape.getVertices().size();
        for(int i = 0; i < edgeCount; ++i)
        {
            int nei = _nextEdge(i, edgeCount);

            //edge normal
            cvVec2f ep0 = verts[i];
            cvVec2f ep1 = verts[nei];
            cvVec2f ev = ep1 - ep0;
            cvVec2f en = ev.computePerpendicular();
            en.normalize();
            cvVec2f enlocal = en;
            //edge normal in local of shape B
            a2b.transformVector(enlocal);
            // ep0 in local of B
            cvVec2f ep0w = a2b * ep0;
            float p = _getPeneDist(enlocal, ep0w, shape);

            if(p > 0)
            {
                return p;
            }

            if (p > smallestPen)
            {
                smallestPen = p;
                minEdgeIndx = i;
                enormal = en;
            }
        }

        return smallestPen;
    }

    float _findIncidentEdge(const cvPolygonShape& shape, const cvVec2f& refNormal,
            const cvMat33& trans, int& incidentEdgeIndex)
    {
        //find edge whose normal has smallest dot value with ref normal
        float maxD = FLT_MAX;
        auto& verts = shape.getVertices();
        for(int i = 0;i < verts.size(); ++i)
        {
            cvVec2f e0 = verts[i];
            cvVec2f e1 = verts[_nextEdge(i, verts.size())];
            cvVec2f e = e1 - e0;
            cvVec2f en = e.computePerpendicular();

            float d = refNormal.dot(en);
            if(d < maxD)
            {
                incidentEdgeIndex = i;
                maxD = d;
            }
        }

        return maxD;
    }

    SATResult _polyToPoly(const cvPolygonShape& shapeA, const cvPolygonShape& shapeB,
            const cvMat33& matA, const cvMat33& matB)
    {
        SATResult res;

        int minEdgeIdxA = 0;
        int minEdgeIdxB = 0;
        cvVec2f edgeNormalA;
        cvVec2f edgeNormalB;

        //for all edges of shape A
        float minPenA = _getMinPenD(shapeA, shapeB, matA, matB, minEdgeIdxA, edgeNormalA);
        float minPenB = _getMinPenD(shapeB, shapeA, matB, matA, minEdgeIdxB, edgeNormalB);

        if(minPenA > 0 || minPenB > 0) //has sep axis 
            return res;

        int refEdgeIdx;
        int neEdgeIdx0; //neighbour of ref edge
        int neEdgeIdx1; //neighbour of ref edge
        int incidentEdgeIdx;

        cvVec2f sepNormal;

        // incident edeg point
        cvVec2f incEp[2];
        cvVec2f refEp[2];
        //neighbor edge pt on ref shape
        cvVec2f neibEp0[2];
        cvVec2f neibEp1[2];
        cvVec2f neibN[2];
        const cvPolygonShape* incShape = nullptr;
        const cvPolygonShape* refShape = nullptr;

        cvMat33 refMat;
        cvMat33 incMat;;

        // work on sep axis with smallest penetration
        if(minPenA > minPenB)
        {
            incShape = &shapeB;
            incMat = matB;
            refShape = &shapeA;
            refMat = matA;

            sepNormal = edgeNormalA;
            refEdgeIdx = minEdgeIdxA;

        }
        else
        {
            incShape = &shapeA;
            incMat = matA;
            refShape = &shapeB;
            refMat = matB;

            sepNormal = edgeNormalB;
            refEdgeIdx = minEdgeIdxB;
        }

        // compute incident edge, and neihbough edges 
        neEdgeIdx0 = _prevEdge(refEdgeIdx, (*refShape).getVertices().size());
        neEdgeIdx1 = _nextEdge(refEdgeIdx, (*refShape).getVertices().size());

        _findIncidentEdge((*incShape), sepNormal, incMat, incidentEdgeIdx);
        auto& incVert = (*incShape).getVertices();
        auto& refVert = (*refShape).getVertices();

        incEp[0] = incMat * incVert[incidentEdgeIdx];
        incEp[1] = incMat * incVert[_nextEdge(incidentEdgeIdx, incVert.size())];
        refEp[0] = refMat * refVert[refEdgeIdx];
        refEp[1] = refMat * refVert[_nextEdge(refEdgeIdx, refVert.size())];
        neibEp0[0] = refMat * refVert[neEdgeIdx0];
        neibEp0[1] = refMat * refVert[_nextEdge(neEdgeIdx0, refVert.size())];
        neibEp1[0] = refMat * refVert[neEdgeIdx1];
        neibEp1[1] = refMat * refVert[_nextEdge(neEdgeIdx1, refVert.size())];

        neibN[0] = neibEp0[1] - neibEp0[0];
        neibN[0] = neibN[0].computePerpendicular();
        neibN[0].normalize();
        neibN[1] = neibEp1[1] - neibEp1[0];
        neibN[1] = neibN[1].computePerpendicular();
        neibN[1].normalize();

        // clipping 
        _clipEdgeWithNormal(incEp[0], incEp[1], neibN[0], neibEp0[0]);
        _clipEdgeWithNormal(incEp[0], incEp[1], neibN[1], neibEp1[1]);

        res.numPt = 0;
        res.normal = sepNormal;
        // find out edge points below ref plane
        for(int i = 0; i < 2; ++i)
        {
            float d = (incEp[i] - refEp[0]).dot(sepNormal);
            if(d < 0)
            {
                res.point[res.numPt] = incEp[i] - sepNormal * d;
                res.distance[res.numPt] = d;
                res.numPt++;
            }
        }

        return res;
    }
}
