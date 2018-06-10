#pragma once

#include <core/cvMath.h>
#include "cvDistance.h"
#include "cvQuery.h"
#include <collision/cvCollisionDef.h>

class cvCircle;
class cvPolygonShape;
class cvConvexShape;

namespace  SAT
{
    using namespace cvCol;

	struct SATPt
	{
        cvVec2f point;
        float distance;
        cvFeatureType featureTypes[2];
        cvFeatureId featureIds[2];
	};

    struct SATResult
    {
        int numPt = 0;
		SATPt pts[2];
        cvVec2f normal;
        bool penetrated = false;
    };

    SATResult _circleToPolygon(const cvCircle& circle, const cvPolygonShape& poly,
            const cvMat33& transA, const cvMat33& transB);

    SATResult _pointToPolygon(const cvVec2f& pt, const cvPolygonShape& poly, const cvMat33& trans);

    SATResult _polyToPoly(const cvPolygonShape& shapeA, const cvPolygonShape& shapeB,
            const cvMat33& matA, const cvMat33& matB);
}
