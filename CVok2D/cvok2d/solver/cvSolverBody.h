#pragma once
#include <core/cvMath.h>

struct cvSolverBody
{
    cvTransform m_transform;
    cvVec3f m_velocity; //first 2 components as linear velocity, third as angular
    cvVec3f m_posVelocity; // pseudo vel for fixing position error
};
