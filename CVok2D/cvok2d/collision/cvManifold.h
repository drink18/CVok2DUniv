#pragma once

#include <world/cvBody.h>
#include <core/cvMath.h>

// pair that contains all data need to perform NP collision detection
// on a broadphase pair. From each broadphase pair we generate a NP Pair 
// which is consumed by NP pipeline.
class cvNPPair
{
public:
    cvBodyId m_bodyA;
    cvBodyId m_bodyB;
    cvMat33 m_transA;
    cvMat33 m_transB;

    cvShape* m_shapeA;
    cvShape* m_shapeB;
};

struct cvManifoldPoint
{
    cvVec2f m_point;
    cvVec2f m_normal;
    float m_distance;
};

// narrow phase collision detection result. 1 or more manifolds per narrow phase 
// pair
struct cvManifold
{
    static constexpr int MAX_MANIFOLD_POINT = 2;

    cvBodyId m_bodyA;
    cvBodyId m_bodyB;

    cvManifoldPoint m_points[MAX_MANIFOLD_POINT];
    cvVec2f m_normal;
    int m_numPt = 0; //number of point in manfild

};
