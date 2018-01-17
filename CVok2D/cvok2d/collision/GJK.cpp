#include "GJK.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>

namespace GJK
{
    using namespace std;
    using namespace cvDist;


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
        auto lambda = [&](const cvVec2f& d){return shape.getSupport(d);};
        return _pointToConvex(queryPt, lambda);
    }

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input)
    {
        cvMat33 mat;
        input.shapeXForm.toMat33(mat);
        cvMat33 inv;
        mat.getInvert(inv);

        cvVec2f q0 = inv * input.q;
        auto res = pointToConvex(q0, *input.shape);

        if(res.result == GJKResult::GJK_GOOD)
            res.closetPt = mat * res.closetPt;
        return res;
    }
}
