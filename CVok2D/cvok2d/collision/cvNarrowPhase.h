#pragma once

#include <vector>
#include <world/cvBody.h>
#include <collision/cvManifold.h>

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
