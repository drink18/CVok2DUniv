#include "cvISimulationControl.h"
#include <vector>
#include <memory>

#include "cvSimulationContext.h"

class cvSimulationControlSimple : public cvISimulationControl
{
public:
    cvSimulationControlSimple(cvBroadphase* bp, cvSimulationContext* simCtx)
        :m_bp(bp)
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
};
