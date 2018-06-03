#pragma  once

#include <vector>
#include <unordered_map>
#include "cvCollisionAgent.h"
#include <collision/cvManifold.h>
#include <memory>

struct cvSimulationContext
{
	typedef std::unordered_map<cvNPPair, cvManifold> NPPairs;
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
    NPPairs m_NpPairs;
    std::vector<cvManifold*> m_Manifolds;
    uint32_t m_solverIterCount = 6;
};
