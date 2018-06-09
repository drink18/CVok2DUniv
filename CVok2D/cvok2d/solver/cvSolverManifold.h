#pragma once

#include <world/cvBody.h>
#include <core/cvMath.h>
#include <core/cvHashUtils.h>


struct cvManifoldPoint
{
    cvVec2f m_point;
    float m_distance = 0;
    float m_normalImpl = 0;
    float m_tangentImpl = 0;
};

struct cvSolverManifold
{
    static constexpr int MAX_MANIFOLD_POINT = 2;

    cvBodyId m_bodyA;
    cvBodyId m_bodyB;
    uint32_t m_solverBodyA;
    uint32_t m_solverBodyB;

    cvManifoldPoint m_points[MAX_MANIFOLD_POINT];
    cvVec2f m_normal;
    int m_numPt = 0; //number of point in manfild

    float m_friction = 0.2f;
    float m_restitution = 0.05f;
    float m_rollingFriction = 0.00f;
};
