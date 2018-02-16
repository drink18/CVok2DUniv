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
}

void cvSimulationControlSimple::narrowPhase()
{
}

void cvSimulationControlSimple::postCollide()
{
}
