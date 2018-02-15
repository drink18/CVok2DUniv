#pragma  once

#include <vector>
#include "cvCollisionAgent.h"

struct cvSimulationContext
{
    std::vector<cvCollisionAgent*> m_colAgents;
};
