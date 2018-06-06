#pragma once
#include <core/cvMath.h>

struct cvSolverBody
{
    cvTransform m_transform;
    cvVec3f m_velocity; //first 2 components as linear velocity, third as angular
    cvVec3f m_posVelocity; // pseudo vel for fixing position error
    float m_accumImpl;

    void applyImpulse(cvVec3f& vel, const cvVec3f& J,  const cvVec3f& em, float lambda)
    {
        vel += J * em * lambda;
    }

    void applyVelImp(const cvVec3f& J, const cvVec3f& em, float lambda)
    {
        applyImpulse(m_velocity, J, em, lambda);
    }

    void applyPosImp(const cvVec3f& J, const cvVec3f& em, float lambda)
    {
        applyImpulse(m_posVelocity, J, em, lambda);
    }
};
