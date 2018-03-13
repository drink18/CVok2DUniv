#pragma  once

#include <core/cvMath.h>
#include <world/cvBody.h>

struct cvContact
{
    cvBodyId bodyIds[2];

    cvVec2f m_dpoint;
    cvVec2f m_dnormal;
    float   m_distance;
};
