#pragma once
#include <core/cvMath.h>
#include <collision/cvManifold.h>

struct cvContactConstraint
{
    //jacobian
    cvVec3f JA;
    cvVec3f JB;

    cvVec3f tJA;
    cvVec3f tJB;

    // rolling frction
    cvVec3f rJA;
    cvVec3f rJB;

    cvVec3f MA; //mass matrice
    cvVec3f MB; //mass matrice

    //solver body id
    int bodyAId;
    int bodyBId;

    float bias = 0;
    float posBias = 0;

    cvManifold* m_manifold;
    uint8_t m_maniPtIdx;

    float m_accumImpl = 0;
    float m_tangentImpl = 0;
    float m_posAccumImp = 0;

    float m_friction = 0;
    float m_restitution = 0;
    float m_rollingFriction = 0;
};
