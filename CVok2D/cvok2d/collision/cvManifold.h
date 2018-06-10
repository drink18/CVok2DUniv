#pragma once

#include <world/cvBody.h>
#include <core/cvMath.h>
#include <core/cvHashUtils.h>
#include <solver/cvSolverManifold.h>
#include <collision/cvCollisionDef.h>

typedef uint16_t cvShapeKey;

struct cvManifoldPtFeature
{
    cvCol::cvFeatureType m_typeA = cvCol::MF_Vertex;
    cvCol::cvFeatureType m_typeB = cvCol::MF_Edge;
    cvCol::cvFeatureId m_featureA = 0;
    cvCol::cvFeatureId m_featureB = 0;

    void init(cvCol::cvFeatureType typeA, cvCol::cvFeatureType typeB, 
            cvCol::cvFeatureId featureA, cvCol::cvFeatureId featureB)
    {
        m_featureA = featureA;
        m_featureB = featureB;
        m_typeA = typeA;
        m_typeB = typeB;
    }
};

// narrow phase collision detection result. 1 or more manifolds per narrow phase 
// pair
struct cvManifold
{
    static constexpr int MAX_MANIFOLD_POINT = 2;

    cvBodyId m_bodyA;
    cvBodyId m_bodyB;
    cvShapeKey   m_shapeKeyA;
    cvShapeKey   m_shapeKeyB;

    cvManifoldPoint m_points[MAX_MANIFOLD_POINT];
    cvVec2f m_normal;
    int m_numPt = 0; //number of point in manfild

    float m_friction = 0.2f;
    float m_restitution = 0.05f;
    float m_rollingFriction = 0.00f;

    uint32_t solveManifoldIdx = 0;
    cvManifoldPtFeature m_feature;

    bool matchManifold(const cvManifold& m);
    void init(const cvBody& bodyA, const cvBody& bodyB);
};


