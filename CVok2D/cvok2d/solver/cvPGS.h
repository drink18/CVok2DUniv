#pragma once

#include "cvConstraint.h"
#include "cvSolverBody.h"
#include <vector>

class cvWorld;
class cvPGSSolver
{
public:
    void solve();

    void solveContacts();
    void setupSolverBodies(cvWorld& world);
    void setupContratins(const std::vector<cvManifold>& manifolds, const cvWorld& world);
    void finishSolver(cvWorld& world);

    std::vector<cvSolverBody> m_solverBodies;
    std::vector<cvContactConstraint> m_ContactContraints;
};
