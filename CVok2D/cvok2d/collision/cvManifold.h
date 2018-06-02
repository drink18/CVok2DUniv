#pragma once

#include <world/cvBody.h>
#include <core/cvMath.h>

// pair that contains all data need to perform NP collision detection
// on a broadphase pair. From each broadphase pair we generate a NP Pair 
// which is consumed by NP pipeline.
class cvNPPair
{
public:
    cvBodyId m_bodyA;
    cvBodyId m_bodyB;
    cvMat33 m_transA;
    cvMat33 m_transB;

    cvShape* m_shapeA;
    cvShape* m_shapeB;
};

struct cvManifoldPtFeature
{
	enum FeatureType
	{
		MF_Vertex,
		MF_Edge
	};

	FeatureType m_typeA = MF_Vertex;
	FeatureType m_typeB = MF_Edge;
	int m_featureA = 0;
	int m_featureB = 0;

	void init(FeatureType typeA, FeatureType typeB, int featureA, int featureB)
	{
		m_featureA = featureA;
		m_featureB = featureB;
		m_typeA = typeA;
		m_typeB = typeB;
	}
};

struct cvManifoldPoint
{
    cvVec2f m_point;
	float m_distance = 0;
	cvManifoldPtFeature m_feature;
};

// narrow phase collision detection result. 1 or more manifolds per narrow phase 
// pair
struct cvManifold
{
    static constexpr int MAX_MANIFOLD_POINT = 2;

    cvBodyId m_bodyA;
    cvBodyId m_bodyB;

    cvManifoldPoint m_points[MAX_MANIFOLD_POINT];
    cvVec2f m_normal;
    int m_numPt = 0; //number of point in manfild

    float m_friction = 0.2f;
    float m_restitution = 0.05f;
    float m_rollingFriction = 0.00f;
};
