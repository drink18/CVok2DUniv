#pragma once
#include <core/cvMath.h>
#include <collision/cvManifold.h>

struct cvContactConstraint
{
    //jacobian
    cvVec3f JA;
    cvVec3f JB;

    cvVec3f MA; //mass matrice
    cvVec3f MB; //mass matrice

    //solver body id
    int bodyAId;
    int bodyBId;

    float bias;

    cvManifold manifold;
};
