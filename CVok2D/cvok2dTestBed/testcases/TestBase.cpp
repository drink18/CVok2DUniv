#include "TestBase.h"

#include <cvok2d.h>
#include <world/cvWorld.h>
#include "DebugDraw.h"


std::vector<TestInfo>* g_RegisteredTests;

TestBase::TestBase()
{
    cvWorldCInfo cInfo;
    m_world.reset(new cvWorld(cInfo));
}

void TestBase::tick(cvDebugDraw& gdbDraw, float dt)
{

}

void TestBase::setSolverIteration(int n )
{
    m_world->getSimContext().m_solverIterCount = n;
}

int TestBase::getSolverIteration()
{
    return m_world->getSimContext().m_solverIterCount;
}

