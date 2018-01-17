#pragma once

#include <core/cvMath.h>
#include <shape/cvShape.h>
#include "cvDistance.h"
#include <memory>
#include <vector>

class cvShape;
class cvConvexShape;

namespace GJK
{
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
                if(v.index == vtx.index)
                    return true;
            } 
            return false;
        }

        int count() const {return verts.size();}
    };
    struct GJKResult
    {
        enum Result {
            GJK_GOOD,
            GJK_OVERLAP
        };

        Result result;
        cvVec2f closetPt;
        cvVec2f normal;
        float distance;
    };

    struct GJKContext
    {
        const cvConvexShape& shapeA;
        const cvConvexShape& shapeB;
        cvMat33 poseA;
        cvMat33 poseB;
    };

    struct cvPointQueryInput
    {
        cvVec2f q;
        std::shared_ptr<cvConvexShape> shape;
        cvTransform shapeXForm;
    };

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance);
    cvVec2f _getDFromEdge(const cvVec2f& q, const cvVec2f& a, const cvVec2f& b);

    template<typename GET_SUPPORT_FUNC>
    GJKResult _pointToConvex(const cvVec2f& queryPt, const GET_SUPPORT_FUNC& getSupportFunc)
    {
        auto initSupport = getSupportFunc(cvVec2f(-1.0f, 0));
        Simplex simplex;

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
                auto support = getSupportFunc(d);

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
                
                //compute distance to 1 simplex for edge overlap
                auto p2LRes = pointDistanceToLine(queryPt, a.p, b.p);
                if((p2LRes.pt - queryPt).sqrLength() < CV_FLOAT_EPS)
                    break;

                cvVec2f d = _getDFromEdge(queryPt, a.p, b.p);
                //get support
                auto support = getSupportFunc(d);

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
                        auto support = getSupportFunc(d);
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
                        auto support = getSupportFunc(d);
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
                        auto support = getSupportFunc(d);
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

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input);
}
