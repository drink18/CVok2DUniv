#pragma  once

#include <vector>
#include "cvCollisionAgent.h"
#include <collision/cvManifold.h>
#include <memory>

struct cvSimulationContext
{
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
    std::vector<cvNPPair>  m_NpPairs;
    std::vector<cvManifold> m_Manifolds;
    uint32_t m_solverIterCount = 6;
};
