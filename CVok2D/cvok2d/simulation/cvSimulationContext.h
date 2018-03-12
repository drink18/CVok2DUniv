#pragma  once

#include <vector>
#include "cvCollisionAgent.h"
#include "cvNPData.h"
#include <memory>

struct cvSimulationContext
{
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
    std::vector<cvNPPair>  m_NpPairs;
};
