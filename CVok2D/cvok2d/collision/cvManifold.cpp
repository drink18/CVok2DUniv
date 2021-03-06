#include "cvManifold.h"

#include <vector>
#include <world/cvWorld.h>
#include <world/cvMaterial.h>
#include <shape/cvShape.h>
#include <shape/cvCompoundShape.h>
#include <collision/cvCollisionDispatch.h>

bool cvManifoldPoint::matchPoint(cvCol::cvFeatureType types[2], cvCol::cvFeatureId ids[2])
{
    return ((types[0] == m_featureTypes[0] && types[1] == m_featureTypes[1])
        || (types[1] == m_featureTypes[0] && types[0] == m_featureTypes[1]))
        && ((ids[0] == m_featureIds[0] && ids[1] == m_featureIds[1])
            || (ids[1] == m_featureIds[0] && ids[0] == m_featureIds[1]));
}

bool cvManifold::matchManifold(const cvManifold& m)
{
    // no need to match body id, as they come from narrow phase pair hence guarenteed to be identical
    return (m_shapeKeyA == m.m_shapeKeyA && m.m_shapeKeyB == m_shapeKeyB);
}

void cvManifold::init(const cvBody& bodyA, const cvBody& bodyB)
{
    m_bodyA = bodyA.getBodyId();
    m_bodyB = bodyB.getBodyId();
    auto matA = bodyA.getMaterial();
    auto matB = bodyB.getMaterial();
    if (matA != nullptr && matB != nullptr)
    {
        m_friction = cvMaterial::CombineFirction(*matA, *matB);
        m_restitution = cvMaterial::CombineRestituion(*matA, *matB);
        m_rollingFriction = cvMaterial::CombineRollingFriction(*matA, *matB);
    }
    else if (matA != nullptr)
    {
        m_friction = matA->m_friction;
        m_restitution = matA->m_restitution;
        m_rollingFriction = matA->m_rollingFriction;
    }
    else if (matB != nullptr)
    {
        m_friction = matB->m_friction;
        m_restitution = matB->m_restitution;
        m_rollingFriction = matB->m_rollingFriction;
    }
}
