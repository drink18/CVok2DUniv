#include "cvManifold.h"

#include <vector>
#include <world/cvWorld.h>
#include <shape/cvShape.h>
#include <shape/cvCompoundShape.h>
#include <collision/cvCollisionDispatch.h>


bool cvManifold::matchManifold(const cvManifold& m)
{
    // no need to match body id, as they come from narrow phase pair hence guarenteed to be identical
    return (m_shapeKeyA == m.m_shapeKeyA && m_shapeKeyB == m_shapeKeyB);
}
