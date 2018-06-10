#pragma once

#include <core/cvMath.h>
#include "cvQuery.h"
#include "cvDistance.h"
#include <collision/cvCollisionDef.h>
#include <memory>

class cvShape;
class cvConvexShape;

namespace GJK
{
    struct SimplexVertex
    {
        cvVec2f p;
        // for point to shape, sA is always query point
        cvVec2f sA;
        cvVec2f sB;

        // support vertex index
        int indexA;  //always 0 if point to shape query
        int indexB; 

        bool operator==(const SimplexVertex& other) const 
        {
            return indexA == other.indexA && indexB == other.indexB;
        }
    };

    using namespace cvDist;
    struct GJKResult
    {
        enum Result {
            GJK_GOOD,
            GJK_OVERLAP
        };

        Result result = GJK_OVERLAP;
        cvVec2f closetPt;
        cvVec2f normal;
        float distance;
        cvCol::cvFeatureType featureType;
        cvCol::cvFeatureId featureId;
    };

    struct cvPointQueryInput
    {
        cvVec2f q;
        const cvConvexShape& shape;
        cvMat33 shapeXForm;

        cvPointQueryInput(cvVec2f pt, const cvConvexShape& cvx, const cvMat33& xform)
            : q(pt), shape(cvx), shapeXForm(xform) {}
    };

    struct cvConvex2ConvexGJKResult
    {
        cvVec2f m_pA;
        cvVec2f m_pB;
        cvVec2f m_seperation;
        float m_distance;
        bool m_succeed = false;
    };

    GJKResult pointToConvex(const cvVec2f& queryPt, const cvConvexShape& shape);

    GJKResult cvPointToConvexShape(const cvPointQueryInput& input);

    cvConvex2ConvexGJKResult cvGJKConvexToConvex(const cvShapeQueryInput& input);

    SimplexVertex _getSupportOnMinkowsiDiff(const cvShapeQueryInput& input, const cvVec2f& d);

}
