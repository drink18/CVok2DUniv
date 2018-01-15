#include "WorldIntegration.h"

#include <shape/cvShape.h>
#include <shape/cvPolygonShape.h>
#include <simulation/cvWorld.h>
#include <DebugDraw.h>

using namespace std;

WorldIntegration::WorldIntegration()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    shared_ptr<cvShape> shape(
            cvPolygonShape::createBox(cvVec2f(2.0f, 2.0f), 0.05f));
    cvBodyCInfo bodyInfo;
    bodyInfo.m_mass = 1.0f;
    bodyInfo.m_shape = shape;

    cvBodyId id = m_world->createBody(bodyInfo, true);
    m_world->setBodyAngularVelocity(id, 1.0f);
}

void WorldIntegration::tick(cvDebugDraw& debugDraw)
{
    m_world->integrate(0.06f);
    debugDraw.DrawWorld(*m_world);
}
