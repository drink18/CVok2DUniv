#pragma once

#include <world/cvBody.h>
#include <core/cvMath.h>
#include <core/cvHashUtils.h>
#include <solver/cvSolverManifold.h>

typedef uint16_t cvShapeKey;

struct cvManifoldPtFeature
{
    enum FeatureType
    {
        MF_Vertex,
        MF_Edge
    };

    FeatureType m_typeA = MF_Vertex;
    FeatureType m_typeB = MF_Edge;
    int m_featureA = 0;
    int m_featureB = 0;

    void init(FeatureType typeA, FeatureType typeB, int featureA, int featureB)
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

    cvManifoldPoint m_points[MAX_MANIFOLD_POINT];
    cvVec2f m_normal;
    int m_numPt = 0; //number of point in manfild

    cvShapeKey m_shapeKeyA;
    cvShapeKey m_shapeKeyB;

    float m_friction = 0.2f;
    float m_restitution = 0.05f;
    float m_rollingFriction = 0.00f;

    uint32_t solveManifoldIdx = 0;
    cvManifoldPtFeature m_feature;
};

// pair that contains all data need to perform NP collision detection
// on a broadphase pair. From each broadphase pair we generate a NP Pair 
// which is consumed by NP pipeline.
class cvNPPair
{
public:
    cvBodyId m_bodyA;
    cvBodyId m_bodyB;

    std::vector<cvManifold> m_manifolds;

    cvNPPair(cvBodyId bodyA, cvBodyId bodyB)
    {
        if(bodyA.getVal() > bodyB.getVal())
        {
            m_bodyA = bodyB;
            m_bodyB = bodyA;
        }
        else
        {
            m_bodyA = bodyA;
            m_bodyB = bodyB;
        }
    }

    size_t getHash() const
    {
        size_t h = 0;
        cvHash::hash_combine(h, m_bodyA.getVal(), m_bodyB.getVal());
        return h;
    }

    bool operator==(const cvNPPair& other) const
    {
        return getHash() == other.getHash();
    }

    // re-compuate all possible manifolds in a pair
    void EvaluateManifolds(cvWorld& world);
};

namespace std
{
    template<> struct hash<cvNPPair>
    {
        size_t operator()(const cvNPPair& p) const
        {
            return p.getHash();
        }
    };
}

