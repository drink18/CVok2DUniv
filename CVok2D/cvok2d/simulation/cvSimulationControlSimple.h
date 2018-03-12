#include "cvISimulationControl.h"
#include <vector>
#include <memory>

#include "cvSimulationContext.h"

class cvWorld;

class cvSimulationControlSimple : public cvISimulationControl
{
public:
    cvSimulationControlSimple(cvBroadphase* bp, cvSimulationContext* simCtx, cvWorld* world)
        :m_bp(bp), m_simContext(simCtx), m_world(world)
    {
    }

    virtual ~cvSimulationControlSimple(){};

    virtual void preCollide() override;
    virtual void updateBP() override; //generate collison agent for narrow phase
    virtual void narrowPhase() override;
    virtual void postCollide() override;

public:
    // simulation data
    std::shared_ptr<cvSimulationContext> m_simContext;
private:
    cvBroadphase* m_bp;
    cvWorld* m_world;
};
