#include "cvMotion.h"
#include <world/cvBody.h>
#include <shape/cvCircle.h>
#include <shape/cvPolygonShape.h>

cvMotionId const cvMotion::StaticMotionId = cvMotionId(0);

void cvMotion::InitializeMotion(cvMotion& motion, cvMotion::MotionType mt, const cvBodyCInfo& cinfo)
{
    cvAssertMsg(mt != cvMotion::MotionType::Static, "Should not be called on static bodies");
    motion.m_transform = cinfo.m_initTransform;
    motion.m_angularVel = 0;
    motion.m_linearVel = cvVec2f::getZero();

    motion.m_invMassAndInertia = cvVec2f::getZero();
    if(mt == cvMotion::MotionType::Dynamic)
    {
        float invMass = 1.0f / cinfo.m_mass;
        float invInertia = 1;
        if(cinfo.m_shape->getShapeType() == cvShape::eCircle)
        {
            auto& circle = static_cast<cvCircle&>(*cinfo.m_shape);
            float r = circle.getRadius();
            float inertia = 0.5f * cinfo.m_mass * r * r;
            invInertia = 1.0f / inertia;
        }
        if(cinfo.m_shape->getShapeType() == cvShape::ePolygon)
        {
            auto& poly = static_cast<cvPolygonShape&>(*cinfo.m_shape);
            const cvAabb& aabb = poly.getAabb();
            float extX = aabb.m_Max.x - aabb.m_Min.x;
            float extY = aabb.m_Max.y - aabb.m_Min.y;
            float inertia = 1.0f / 12.0f * cinfo.m_mass * (extX * extX + extY * extY);
            invInertia = 1.0f / inertia;
        }
        motion.m_invMassAndInertia.set(invMass, invInertia);
    }
}

