#pragma  once

#include <vector>
#include "cvCollisionAgent.h"
#include <memory>

struct cvSimulationContext
{
    std::vector<std::unique_ptr<cvCollisionAgent>> m_colAgents;
};
