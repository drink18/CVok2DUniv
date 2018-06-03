#pragma once

#include "cvConstraint.h"
#include "cvSolverBody.h"
#include <vector>

class cvWorld;
struct cvStepInfo;
struct cvSimulationContext;

class cvPGSSolver
{
public:
    void solve();

    void solveContacts(int nIter);
    void setupSolverBodies(cvWorld& world);
    void setupContactConstraints(const std::vector<cvManifold*> &manifolds,
                                 const cvWorld &world, const cvSimulationContext &simCtx,
                                 const cvStepInfo &stepInfo);

    void setupFrictionConstraints(const std::vector<cvManifold*> &manifolds,
                                 const cvWorld &world, const cvSimulationContext &simCtx,
                                 const cvStepInfo &stepInfo);

    void finishSolver(cvWorld& world, const cvStepInfo &stepInfo);

    std::vector<cvSolverBody> m_solverBodies;
    std::vector<cvContactConstraint> m_ContactContraints;

protected:
    void solvePenetrations();
    void solvePositionErr();
    void solveFriction();
    void solveRollingFriction();
};
