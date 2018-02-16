#include "cvSimulationControlSimple.h"
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
        cvCollisionAgent* ag = new cvCollisionAgent();
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);
        ag->m_bodyIds[0] = node0->m_bodyId;
        ag->m_bodyIds[1] = node1->m_bodyId;

        m_simContext->m_colAgents.push_back(ag);
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
}

void cvSimulationControlSimple::narrowPhase()
{
}

void cvSimulationControlSimple::postCollide()
{
}
