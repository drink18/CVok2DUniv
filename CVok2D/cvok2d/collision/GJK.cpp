#include "GJK.h"

#include <shape/cvShape.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>

namespace GJK
{
    using namespace std;

    struct Simplex
    {
        vector<SimplexVertex> verts;

        Simplex(){}

        void addVertex(SimplexVertex& v)
        {
            if(verts.size() < 3)
            {
                verts.push_back(v);
            }
        }
        int count;
    };

    cvVec2f getPointOnMinkovskiSubEdge(const cvConvexShape& shapeA, const cvConvexShape& shapeB, 
            const cvMat33& a2b, const cvVec2f& d)
    {
        cvVec2f p1 = shapeA.getSupport(d);
        cvVec2f dInB = d;
        a2b.transformVector(dInB);
        cvVec2f p2 = shapeB.getSupport(-dInB);
        return p1 - p2;
    }

    GJKResult doGJK(const GJKContext& ctx, cvVec2f& pointA, cvVec2f& pointB, cvVec2f& normal, float distance)
    {
        GJKResult res = GJKResult::Separated;
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

    float pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape)
    {
        auto& vertices = shape.getVertices();

        Simplex simplex;
        SimplexVertex v(vertices[0], 0, 0.0f);

        simplex.addVertex(v);

        // build a search vector d
        cvVec2f d = queryPt - vertices[0];

        //get support point
        auto support = shape.getSupport(d);
        //simplex.addVertex(support);
		return 0;

    }
}
