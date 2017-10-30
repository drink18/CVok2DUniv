#pragma once

#include <core/cvMath.h>

typedef std::uint16_t cvMotionId;

class cvMotion
{
public:
    enum Type
    {
        MO_Static,
        MO_Dynamic
    };
public:
    cvMotion()
    {
        m_linearVel.set(0, 0);
        m_angularVel = 0;
    }


    cvMotionId getId()const {return m_id;};
    cvVec2f getLinearVel() const {return m_linearVel;}
    float getAngularVel() const {return m_angularVel;}

private:
    std::uint16_t m_id;
    cvVec2f m_linearVel;
    float m_angularVel;
};
