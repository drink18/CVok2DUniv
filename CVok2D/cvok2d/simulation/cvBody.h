#pragma once

#include <memory>
#include <core/cvMath.h>
#include <shape/cvShape.h>

struct cvBodyCInfo
{
	std::shared_ptr<cvShape> m_shape;
	cvTransform m_initTransform;
	float m_mass;
};

class cvBody
{
public:
	cvBody(cvBodyCInfo& cinfo);

	const cvTransform& getTransform() const { return m_transform; }
	bool isStatic() const;
	bool isDynamic() const;
		
private:
	cvTransform m_transform;
	std::shared_ptr<cvShape> m_shape;
	cvVec2f m_mass;
};