#pragma once

#include "cvConstraint.h"
#include "cvSolverBody.h"
#include <vector>

class cvWorld;
struct cvStepInfo;
struct cvSimulationContext;

struct cvSimulationContext;

class cvPGSSolver
{
public:
    void solve();

    void solveContacts(cvSimulationContext& ctx);

    void finishSolver(cvSimulationContext& ctx, cvWorld& world, const cvStepInfo &stepInfo);

protected:
    void solvePenetrations(cvSimulationContext& ctx);
    void solvePositionErr(cvSimulationContext& ctx);
    void solveFriction(cvSimulationContext& ctx);
    void solveRollingFriction(cvSimulationContext& ctx);
};
