#include "GJK.h"

#include <shape/cvShape.h>
#include <shape/cvConvexShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>

namespace GJK
{
    using namespace std;
    using namespace cvDist;

    struct Simplex
    {
        std::vector<SimplexVertex> verts;

        Simplex(){}

        void addVertex(SimplexVertex& v)
        {
            if(verts.size() < 3)
            {
                verts.push_back(v);
                if (count() == 3)
                {
                    cvVec2f va = v.p - verts[0].p;
                    cvVec2f v10 = verts[1].p - verts[0].p;
                    if (va.cross(v10) > 0)
                    {
                        std::swap(verts[0], verts[1]);
                    }
                }
            }
        }

        void removeVtx(int idx)
        {
            verts.erase(verts.begin() + idx);
        }

        bool hasVtx(const SimplexVertex& vtx)
        {
            for(auto& v : verts)
            {
                if(v.indexA == vtx.indexA && v.indexB == vtx.indexB)
                    return true;
            }
            return false;
        }

        size_t count() const {return verts.size();}
    };

    cvVec2f _getDFromEdge(const cvVec2f& q, const cvVec2f& a, const cvVec2f& b)
    {
        cvVec2f ab = a - b;
        cvVec3f ab3 = cvVec3f(ab.x, ab.y, 0);
        cvVec3f z(0, 0, 1);
        cvVec3f d3 = ab3.cross(z);
        cvVec2f d(d3.x, d3.y);

        cvVec2f qa = q - a;
        if (qa.dot(d) < 0) d *= -1;
        return d;
    }

    template<typename GET_SIMPLEX_FUNC>
    GJKResult _pointToConvex(const cvVec2f& queryPt, const GET_SIMPLEX_FUNC& getSimplexFromSupport, Simplex& simplex)
    {
        auto initSupport = getSimplexFromSupport(cvVec2f(-1.0f, 0));

        simplex.addVertex(initSupport);

        while(true)
        {
            if(simplex.count() == 1) 
            {
                // build a search vector d
                cvVec2f d = queryPt - simplex.verts[0].p;

                // vertex overalpping
                if (d.length() < CV_FLOAT_EPS)
                    break;

                //get support point
                auto simP = getSimplexFromSupport(d);

                if (simplex.hasVtx(simP))
                {
                    GJKResult res;
                    res.result = GJKResult::GJK_GOOD;
                    res.closetPt = simP.p;
                    res.distance = (simP.p - queryPt).length();
                    return res;
                }

                // just add current support
                simplex.addVertex(simP);
            }
            else if(simplex.count() == 2)
            {
                auto a = simplex.verts[0];
                auto b = simplex.verts[1];

                //compute distance to 1 simplex for edge overlap
                auto p2LRes = pointDistanceToLine(queryPt, a.p, b.p);
                if((p2LRes.pt - queryPt).sqrLength() < CV_FLOAT_EPS)
                    break;

                cvVec2f d = _getDFromEdge(queryPt, a.p, b.p);
                //get support
                auto simP = getSimplexFromSupport(d);
                cvVec2f sp = simP.p;
                p2LRes = pointDistanceToLine(simP.p, a.p, b.p);

                if(simplex.hasVtx(simP) || (p2LRes.pt - simP.p).sqrLength() < CV_FLOAT_EPS) // duplicate vertex or colinear
                {
                    // terminate
                    auto pt2Line = pointDistanceToLine(queryPt, a.p, b.p);
                    GJKResult res;
                    res.result = GJKResult::GJK_GOOD;
                    res.closetPt = pt2Line.pt;
                    res.distance = (pt2Line.pt - queryPt).length();
                    return res;
                }

                simplex.addVertex(simP);
            }
            else if(simplex.count() == 3)
            {
                auto a = simplex.verts[0];
                auto b = simplex.verts[1];
                auto c = simplex.verts[2];
                auto res = cvDist::pointDistanceToTriangle(queryPt, 
                        a.p, b.p, c.p);

                if(res.featureType == cvPt2TriangleClosestPt::Edge)
                {
                    if(res.feature == cvPt2TriangleClosestPt::Edge_AB)
                    {
                        cvVec2f d = _getDFromEdge(queryPt, a.p, b.p);
                        auto simP = getSimplexFromSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(simP == c) 
                        { 
                            break;
                        }
                        simplex.removeVtx(2);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Edge_BC)
                    {
                        cvVec2f d = _getDFromEdge(queryPt, b.p, c.p);
                        auto simP = getSimplexFromSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(simP == a)
                        { 
                            break;
                        }
                        simplex.removeVtx(0);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Edge_CA)
                    {
                        cvVec2f d = _getDFromEdge(queryPt, c.p, a.p);
                        auto simP = getSimplexFromSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(simP == b)
                        { 
                            break;
                        }
                        simplex.removeVtx(1);
                    }
                }
                else if(res.featureType == cvPt2TriangleClosestPt::Vertex)
                {
                    // terminate
                    GJKResult gjkRes;
                    gjkRes.result = GJKResult::GJK_GOOD;
                    gjkRes.closetPt = res.pt; 
                    gjkRes.distance = (res.pt - queryPt).length();
                    if(res.feature == cvPt2TriangleClosestPt::Vtx_A)
                    {
                        simplex.removeVtx(1);
                        simplex.removeVtx(1);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Vtx_B)
                    {
                        simplex.removeVtx(0);
                        simplex.removeVtx(1);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Vtx_C)
                    {
                        simplex.removeVtx(0);
                        simplex.removeVtx(0);
                    }
                    return gjkRes;
                }
                else if (res.featureType == cvDist::cvPt2TriangleClosestPt::Interior)
                {
                    // terminate: overlapping
                    break;
                }
            }
        }
        GJKResult gjkRes;
        gjkRes.result = GJKResult::GJK_OVERLAP;
        return gjkRes;
    }

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape)
    {
        auto lambda = [&](const cvVec2f& d)
        {
            auto s = shape.getSupport(d); 
            SimplexVertex sv;
            sv.indexA = 0;
            sv.indexB = s.index;
            sv.p = s.p;
            sv.sA = queryPt;
            sv.sB = s.p;
            return sv;
        };
        Simplex simplex;
        return _pointToConvex(queryPt, lambda, simplex);
    }

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input)
    {
        cvMat33 mat = input.shapeXForm;
        cvMat33 inv;
        mat.getInvert(inv);

        cvVec2f q0 = inv * input.q;
        auto res = pointToConvex(q0, input.shape);

        if(res.result == GJKResult::GJK_GOOD)
        {
            res.closetPt = input.shapeXForm * res.closetPt;
            res.normal = input.q - res.closetPt;
            res.normal.normalize();
        }
        return res;
    }

    SimplexVertex _getSupportOnMinkowsiDiff(const cvShapeQueryInput& input, const cvVec2f& d)
    {
        cvMat33 invA; input.poseA.getInvert(invA);
        cvMat33 invB; input.poseB.getInvert(invB);

        cvVec2f dA; invA.transformVector(d, dA);
        cvVec2f dB; invB.transformVector(d, dB);
        const cvConvexShape& cvxA = static_cast<const cvConvexShape&>(input.shapeA);
        const cvConvexShape& cvxB = static_cast<const cvConvexShape&>(input.shapeB);
        auto pA = cvxA.getSupport(d);
        auto pB = cvxB.getSupport(-d);

        pA.p = input.poseA * pA.p;
        pB.p = input.poseB * pB.p;

        SimplexVertex v;
        v.p = pA.p - pB.p;
        v.indexA = pA.index;
        v.indexB = pB.index;
        v.sA = pA.p;
        v.sB = pB.p;

        return v;
    }

    cvConvex2ConvexGJKResult cvGJKConvexToConvex(const cvShapeQueryInput& input)
    {

        auto getSupportFn = [&](const cvVec2f& d)
        {
            return _getSupportOnMinkowsiDiff(input, d);
        };

        Simplex simplex;
        auto ptRes = _pointToConvex(cvVec2f(0,0), getSupportFn, simplex);
        cvConvex2ConvexGJKResult res;
        res.m_distance = ptRes.distance;
        if(ptRes.result == GJKResult::GJK_GOOD)
        {
            res.m_succeed = true;
            switch(simplex.count())
            {
                case 0:
                    cvAssertMsg(false, "GJK successed with null simplex");
                case 1:
                    {
                        res.m_pA = simplex.verts[0].sA;
                        res.m_pB = simplex.verts[0].sB;
                    }
                    break;
                case 2:
                    {
                        auto v0 = simplex.verts[0];
                        auto v1 = simplex.verts[1];
                        cvVec2f a = v0.p;
                        cvVec2f b = v1.p;
                        cvVec2f l = b - a;
                        float lambda2 = a.dot(-l) / l.sqrLength();
                        float lambda1 =  1 - lambda2;
                        if(lambda1 < 0) lambda2 = 1;
                        if(lambda2 < 0) lambda1 = 1;
                        res.m_pA = v0.sA * lambda1 + v1.sA * lambda2;
                        res.m_pB = v0.sB * lambda1 + v1.sB * lambda2;

                    }
                    break;
                default:
                    break;
            }
            res.m_seperation = res.m_pB - res.m_pA;
            res.m_seperation.normalize();
        }

        res.m_succeed = ptRes.result == GJKResult::GJK_GOOD;

        return res;
    }
}
