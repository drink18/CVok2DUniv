#pragma once

#include <core/cvMath.h>

namespace cvDist
{
    struct cvPt2LineClosestPt
    {
        cvVec2f pt; //closest point on line;
        float u; //barycenter coord
        float v; //barycenter coord
    };

    struct cvPt2TriangleClosestPt
    {
        enum FeatureType 
        {
            Edge, Vertex, Interior
        };
        enum Feature
        {
            Vtx_A,
            Vtx_B,
            Vtx_C,
            Edge_AB,
            Edge_BC,
            Edge_CA,
            Inside_Tri
        };

        cvVec2f pt; //closest point on line;
        float u; //barycenter coord
        float v; //barycenter coord
        float w; //barycenter coord

        Feature feature;
        FeatureType featureType;
    };

    cvPt2LineClosestPt pointDistanceToLine(const cvVec2f& queryPt, const cvVec2f& p0, const cvVec2f& p1);

    cvPt2TriangleClosestPt pointDistanceToTriangle(const cvVec2f& q, 
            const cvVec2f& a, const cvVec2f& b, const cvVec2f& c);

     
}
