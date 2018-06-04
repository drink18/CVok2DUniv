#pragma  once

#include <vector>
#include <unordered_set>
#include "cvCollisionAgent.h"
#include <collision/cvManifold.h>
#include <solver/cvSolverManifold.h>
#include <memory>

struct cvSimulationContext
{
	typedef std::unordered_set<cvNPPair> NPPairs;
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
    NPPairs m_NpPairs;
    std::vector<cvSolverManifold> m_Manifolds;
    uint32_t m_solverIterCount = 6;
};
