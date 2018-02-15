#pragma  once

class cvBroadphase;

class cvISimulationControl
{
public:
    virtual void preCollide() = 0;
    virtual void updateBP() = 0;
    virtual void narrowPhase() = 0;
    virtual void postCollide() = 0;
};
