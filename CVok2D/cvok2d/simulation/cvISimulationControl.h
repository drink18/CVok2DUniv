#pragma  once

class cvBroadphase;

struct cvStepInfo
{
    float m_dt;
};

class cvISimulationControl
{
public:
    virtual ~cvISimulationControl() {};
    virtual void simulate(cvStepInfo& sepInfo, struct cvSimulationContext& simCtx) = 0;
};
