#pragma once

#include <core/cvMath.h>
#include <core/cvHandle.h>
#include <functional>

struct cvBodyCInfo;

typedef cvHandle<uint16_t, 0x7FFF> cvMotionId;


class cvMotion
{
public:
    enum class MotionType
    {
        Static,
        Kinematic,
        Dynamic
    };
public:
    static const cvMotionId StaticMotionId;

    cvVec2f getLinearVel() const {return m_linearVel;}
    float getAngularVel() const {return m_angularVel;}

    cvVec2f m_linearVel;
    float m_angularVel = 0;
    cvVec2f m_invMassAndInertia;
    cvTransform m_transform;

    int solverBodyId = -1;

public:
    float getInvMass() const {return m_invMassAndInertia.x;}
    float getInvInertia() const {return m_invMassAndInertia.y;}
    static void InitializeMotion(cvMotion& motion, MotionType mt, const cvBodyCInfo& cinfo);

};
