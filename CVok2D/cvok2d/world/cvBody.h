#pragma once

#include <memory>
#include <core/cvHandle.h>
#include <core/cvMath.h>
#include <shape/cvShape.h>

#include <world/cvHandleDefs.h>
#include <world/cvBroadphase.h>
#include <dyanmic/cvMotion.h>


class cvWorld;
class cvMaterial;

struct cvBodyCInfo
{
    cvMotion::MotionType m_motionType = cvMotion::MotionType::Dynamic;
	std::shared_ptr<cvShape> m_shape;
    std::shared_ptr<cvMaterial> m_material;
	cvTransform m_initTransform;
	float m_mass = 1.0f;
};

class cvBody
{
    friend class cvWorld;

    enum Flags
    {
        bKinematic = 1 << 0,
    };

public:
    cvBody();
	cvBody(cvBodyCInfo& cinfo);

	const cvTransform& getTransform() const { return m_transform; }
    cvTransform& accessTransform() {return m_transform;}


    std::shared_ptr<cvShape> getShape() const {return m_shape;}
    void getAabb(cvAabb& out) const;

    void setBroadphaseHandle(cvBroadphaseHandle handle) {m_broadphaseId = handle;}
    cvBroadphaseHandle getBroadphaseHandle() const {return m_broadphaseId;}

    cvMotionId getMotionId() const {return m_motionId;}

    cvBodyId getBodyId() const {return m_id;}

    bool isStatic() const {return m_motionId == cvMotionId::invalid();}
	bool isDynamic() const {return !isStatic();};
    bool IsKinematic() const {return isFlagSet(bKinematic);}

    void setFlag(Flags f, bool enable);
    bool isFlagSet(Flags f) const {return m_flags & f;}

    std::shared_ptr<cvMaterial> getMaterial() const {return m_material;}
private:
    cvBodyId m_id;
    uint32_t m_flags;
	cvTransform m_transform;
	std::shared_ptr<cvShape> m_shape;
    std::shared_ptr<cvMaterial> m_material;
	cvVec2f m_mass;

    cvBroadphaseHandle m_broadphaseId = cvBroadphaseHandle::invalid();
    cvMotionId m_motionId = cvMotionId::invalid();
};

inline void PrintTo(const cvBodyId& id, ::std::ostream* os)
{
    *os << id.getVal();
}
