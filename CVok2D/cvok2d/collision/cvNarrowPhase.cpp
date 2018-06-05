#include "cvNarrowPhase.h"

#include <vector>
#include <world/cvWorld.h>
#include <shape/cvShape.h>
#include <shape/cvCompoundShape.h>
#include <collision/cvCollisionDispatch.h>

using namespace std;
typedef pair<const cvShape&, cvMat33>  shapePair;

void extractShapePairs(const cvBody& body, vector<shapePair>& pairs)
{
    const cvShape* shape = body.getShape().get();
    cvMat33 mat;
    body.getTransform().toMat33(mat);
    if(shape->getShapeType() == cvShape::eCompoundShape)
    {
        auto* comp = static_cast<const cvCompoundShape*>(shape);
        auto& subShape = comp->getSubshapes();
        for(auto& sub : subShape)
        {
            cvMat33 subMat;
            sub.m_transform.toMat33(subMat);
            pairs.push_back(shapePair(*sub.m_shape, subMat * mat));
        }
    }
    else
    {
        shapePair p(*shape, mat);
        pairs.push_back(p);
    }
}

void cvNPPair::EvaluateManifolds(cvWorld& world)
{
    vector<shapePair> pairsA;
    vector<shapePair> pairsB;

    const cvBody& bodyA = world.getBody(m_bodyA);
    const cvBody& bodyB = world.getBody(m_bodyB);

    extractShapePairs(bodyA, pairsA);
    extractShapePairs(bodyB, pairsB);


    for(int a = 0; a < pairsA.size(); ++a)
    {
        auto& spA = pairsA[a];
        for(int b = 0; b < pairsB.size(); ++b)
        {
            auto& spB = pairsB[b];

            cvManifold nm;
            nm.m_bodyA = m_bodyA;
            nm.m_bodyB = m_bodyB;
            nm.m_shapeKeyA = a;
            nm.m_shapeKeyB = b;

            auto fn = cvGetCollisionFn(spA.first.getShapeType(), spB.first.getShapeType());
            fn(spA.first, spB.first, spA.second, spB.second, nm);

            bool hasMatch = false;
            for (auto& m : m_manifolds)
            {
                if (m.matchManifold(nm))
                {
                    for (int i = 0; i < nm.m_numPt; ++i)
                    {
                        auto& p = m.m_points[i];
                        auto& np = nm.m_points[i];
                        np.m_normalImpl = p.m_normalImpl;
                        np.m_tangentImpl = p.m_tangentImpl;
                    }
                    hasMatch = true;
                    m = nm; //copy over
                }
            }
            // new manifold
            if(!hasMatch)
                m_manifolds.push_back(nm);
        }
    }
}