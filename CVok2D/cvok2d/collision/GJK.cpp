#include "GJK.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>

namespace GJK
{
    using namespace std;
    using namespace cvDist;

    struct Simplex
    {
        vector<SimplexVertex> verts;

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
                if(v.index == vtx.index)
                    return true;
            } 
            return false;
        }

        int count() const {return verts.size();}
    };

    //cvVec2f getPointOnMinkovskiSubEdge(const cvConvexShape& shapeA, const cvConvexShape& shapeB, 
            //const cvMat33& a2b, const cvVec2f& d)
    //{
        //cvVec2f p1 = shapeA.getSupport(d);
        //cvVec2f dInB = d;
        //a2b.transformVector(dInB);
        //cvVec2f p2 = shapeB.getSupport(-dInB);
        //return p1 - p2;
    //}

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance)
    {
        GJKResult res;
        /*
        cvMat33 invA;
        ctx.poseA.getInvert(invA);
        cvMat33 invB;
        ctx.poseB.getInvert(invB);

        cvMat33 a2b = ctx.poseA * invB;

        cvVec2f dir(0, 1);
        cvVec2f dirB = dir;
        a2b.transformVector(dirB);

        Simplex simplex;

        cvVec2f a = getPointOnMinkovskiSubEdge(ctx.shapeA, ctx.shapeB, a2b, dir);
        simplex.addPoint(a);

        cvVec2f b = getPointOnMinkovskiSubEdge(ctx.shapeA, ctx.shapeB, a2b, - dir);
        simplex.addPoint(b);

        cvVec2f ab = b - a;

        */


        return res;
    }

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

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape)
    {
        auto& vertices = shape.getVertices();

        Simplex simplex;
        SimplexVertex v(vertices[0], 0, 0.0f);

        simplex.addVertex(v);

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
                auto support = shape.getSupport(d);

                if (simplex.hasVtx(support))
                {
                    GJKResult res;
                    res.result = GJKResult::GJK_GOOD;
                    res.closetPt = support.p;
                    res.distance = (support.p - queryPt).length();
                    return res;
                }

                // just add current support
                simplex.addVertex(support);
            }
            else if(simplex.count() == 2)
            {
                auto a = simplex.verts[0];
                auto b = simplex.verts[1];
                cvVec2f d = _getDFromEdge(queryPt, a.p, b.p);
                //get support
                auto support = shape.getSupport(d);

                if(simplex.hasVtx(support)) // duplicate vertex
                {
                    // terminate
                    auto pt2Line = pointDistanceToLine(queryPt, a.p, b.p);
                    GJKResult res;
                    res.result = GJKResult::GJK_GOOD;
                    res.closetPt = pt2Line.pt;
                    res.distance = (pt2Line.pt - queryPt).length();
                    return res;
                }

                simplex.addVertex(support);
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
                        auto support = shape.getSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(support.index == c.index) 
                        { 
                            break;
                        }
                        simplex.removeVtx(2);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Edge_BC)
                    {
                        cvVec2f d = _getDFromEdge(queryPt, b.p, c.p);
                        auto support = shape.getSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(support.index == a.index) 
                        { 
                            break;
                        }
                        simplex.removeVtx(0);
                    }
                    else if(res.feature == cvPt2TriangleClosestPt::Edge_CA)
                    {
                        cvVec2f d = _getDFromEdge(queryPt, c.p, a.p);
                        auto support = shape.getSupport(d);
                        // detecting vertex that we are about to remove, terminate, edge overlap
                        if(support.index == b.index) 
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
}
