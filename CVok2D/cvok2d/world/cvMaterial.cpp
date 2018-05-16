#include "cvMaterial.h"

float cvMaterial::CombineFirction(const cvMaterial& matA, const cvMaterial& matB)
{
    return (matA.m_friction + matB.m_friction) * 0.5f;
}

float cvMaterial::CombineRollingFriction(const cvMaterial& matA, const cvMaterial& matB)
{
    return (matA.m_rollingFriction + matB.m_rollingFriction) * 0.5f;
}

float cvMaterial::CombineRestituion(const cvMaterial& matA, const cvMaterial& matB)
{
    return (matA.m_restitution + matB.m_restitution) * 0.5f;
}
