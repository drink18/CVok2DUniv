#pragma  once

#include <vector>
#include <unordered_set>
#include "cvCollisionAgent.h"
#include <collision/cvManifold.h>
#include <collision/cvNarrowPhase.h>
#include <solver/cvSolverManifold.h>
#include <solver/cvSolverBody.h>
#include <solver/cvConstraint.h>
#include <memory>

struct cvSimulationContext
{
	typedef std::unordered_set<cvNPPair> NPPairs;
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
    NPPairs m_NpPairs;
    std::vector<cvSolverBody> m_solverBodies;
    std::vector<cvContactConstraint> m_contactContraints;
    uint32_t m_solverIterCount = 6;
};
