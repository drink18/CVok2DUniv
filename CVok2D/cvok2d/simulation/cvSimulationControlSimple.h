#include "cvISimulationControl.h"
#include <vector>
#include <memory>

#include "cvSimulationContext.h"

class cvWorld;

class cvSimulationControlSimple : public cvISimulationControl
{
public:
    cvSimulationControlSimple(cvBroadphase* bp, cvSimulationContext* simCtx, cvWorld* world)
        :m_bp(bp), m_world(world)
    {
    }

    virtual ~cvSimulationControlSimple(){};

    void preCollide(cvSimulationContext& simCtx);
    void updateBP(cvSimulationContext& simCtx); //generate collison agent for narrow phase
    void narrowPhase(cvSimulationContext& simCtx);
    void postCollide(cvSimulationContext& simCtx);
    void integrate(float dt);
    virtual void simulate(cvStepInfo& stepInfo, cvSimulationContext& simCtx) override;

private:
    cvBroadphase* m_bp;
    cvWorld* m_world;
};
