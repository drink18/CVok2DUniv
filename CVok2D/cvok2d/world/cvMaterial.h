#pragma once
class cvMaterial
{
public:
    float m_friction = 0.2f;
    float m_restitution = 0.0f;
    float m_rollingFriction = 0.1f;

    static float CombineFirction(const cvMaterial& matA, const cvMaterial& matB);
    static float CombineRollingFriction(const cvMaterial& matA, const cvMaterial& matB);
    static float CombineRestituion(const cvMaterial& matA, const cvMaterial& matB);
};
