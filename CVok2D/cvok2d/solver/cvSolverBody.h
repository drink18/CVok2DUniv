#pragma once
#include <core/cvMath.h>

struct cvSolverBody
{
    cvTransform m_transform;
    cvVec3f m_invMassAndInertia;
    cvVec3f m_velocity; //first 2 components as linear velocity, third as angular
    float m_accumImpl;
};
