#include "cvSimulationControlSimple.h"
#include <collision/cvCollisionDispatch.h>
#include <world/cvWorld.h>
#include <vector>

using namespace std;
void cvSimulationControlSimple::preCollide()
{
}

void cvSimulationControlSimple::updateBP()
{
    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> delPairs;
    m_bp->updateDirtyNodes(newPairs, delPairs);

    for(auto& p : newPairs)
    {
        unique_ptr<cvCollisionAgent> ag(new cvCollisionAgent());
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);
        ag->m_bodyIds[0] = node0->m_bodyId;
        ag->m_bodyIds[1] = node1->m_bodyId;

        m_simContext->m_colAgents.push_back(std::move(ag));

    }

    for(int i = delPairs.size() - 1; i >=0; --i)
    {
        auto& p = delPairs[i];
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);
        auto& agents = m_simContext->m_colAgents;
        for(int j = agents.size() - 1; j >= 0; --j)
        {
            auto& agent = agents[j];
            if(node0->m_bodyId == agent->m_bodyIds[0] && node1->m_bodyId == agent->m_bodyIds[1])
                agents.erase(agents.cbegin() + j);
        }
    }

    vector<cvBroadphase::BPPair> allPairs;
    vector<cvNPPair>& npPairs = m_simContext->m_NpPairs;
    npPairs.clear();
    m_bp->getAllPairs(allPairs);
    for(auto& p : allPairs)
    {
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);

        npPairs.resize(npPairs.size() + 1);
        cvNPPair& np = npPairs.back();

        const cvBody& bodyA = m_world->getBody(node0->m_bodyId);
        const cvBody& bodyB = m_world->getBody(node1->m_bodyId);

        np.m_bodyA = node0->m_bodyId;
        np.m_bodyB = node1->m_bodyId;
        m_world->getBodyTransform(np.m_bodyA).toMat33(np.m_transA);
        m_world->getBodyTransform(np.m_bodyB).toMat33(np.m_transB);
        np.m_shapeA = bodyA.getShape().get();
        np.m_shapeB = bodyB.getShape().get();
    }
}

void cvSimulationControlSimple::narrowPhase()
{
    auto& npPairs = m_simContext->m_NpPairs;
    m_simContext->m_Manifolds.clear();
    for(auto& p : npPairs)
    {
        int numPt = 0;
        auto* shapeA = p.m_shapeA;
        auto* shapeB = p.m_shapeB;
        auto fn = g_collisionFunction[shapeA->getShapeType()][shapeB->getShapeType()];
        // check if manifold for pair<shapeA, shapeB> exists, then add / update points
        //fn(*shapeA, *shapeB, p.m_transA, p.m_transB, m_simContext->m_Manifolds);

    }
}

void cvSimulationControlSimple::postCollide()
{
}
