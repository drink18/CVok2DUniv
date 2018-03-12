#pragma once

#include "cvBody.h"

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
