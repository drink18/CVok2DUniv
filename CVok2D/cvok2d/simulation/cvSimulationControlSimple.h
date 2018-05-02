#include "cvISimulationControl.h"
#include <vector>
#include <memory>

#include "cvSimulationContext.h"

class cvPGSSolver;
class cvWorld;

class cvSimulationControlSimple : public cvISimulationControl
{
public:
    cvSimulationControlSimple(cvBroadphase* bp, cvSimulationContext* simCtx, cvWorld* world);

    virtual ~cvSimulationControlSimple(){};

    void preCollide(cvStepInfo& stepInfo, cvSimulationContext& simCtx);
    void updateBP(cvSimulationContext& simCtx); //generate collison agent for narrow phase
    void narrowPhase(cvSimulationContext& simCtx);
    void postCollide(cvSimulationContext& simCtx);
    void integrate(float dt);
    void solve(cvSimulationContext& simCtx, const cvStepInfo& stepInfo);
    virtual void simulate(cvStepInfo& stepInfo, cvSimulationContext& simCtx) override;

private:
    cvBroadphase* m_bp;
    cvWorld* m_world;
    cvPGSSolver* m_solver;
};
