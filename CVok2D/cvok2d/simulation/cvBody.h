#pragma once

#include <memory>
#include <core/cvHandle.h>
#include <core/cvMath.h>
#include <shape/cvShape.h>
#include <dyanmic/cvMotion.h>
#include <simulation/cvBroadphase.h>

typedef cvHandle<std::uint16_t, 0x7FFF> cvBodyId;
namespace std
{
    template<> struct hash<cvBodyId>
    {
        std::size_t operator()(const cvBodyId& h) const
        {
            return hash{}(h.getVal());
        }
    };
}

struct cvBodyCInfo
{
	std::shared_ptr<cvShape> m_shape;
	cvTransform m_initTransform;
	float m_mass = 1.0f;
    cvMotion::Type m_motionType = cvMotion::MO_Static;
};

class cvBody
{
public:
    cvBody();
	cvBody(cvBodyCInfo& cinfo);

	const cvTransform& getTransform() const { return m_transform; }
	bool isStatic() const;
	bool isDynamic() const;

    std::shared_ptr<cvShape> getShape() const {return m_shape;}
    void getAabb(cvAabb& out) const;

    void setBroadphaseHandle(cvBroadphaseHandle handle) {m_broadphaseId = handle;}
    cvBroadphaseHandle getBroadphaseHandle() const {return m_broadphaseId;}

private:
	cvTransform m_transform;
	std::shared_ptr<cvShape> m_shape;
	cvVec2f m_mass;

    cvBroadphaseHandle m_broadphaseId = cvBroadphaseHandle::invalid();
};
