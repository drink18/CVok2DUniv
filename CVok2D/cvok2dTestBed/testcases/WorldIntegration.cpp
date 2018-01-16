#include "WorldIntegration.h"

#include <shape/cvShape.h>
#include <shape/cvPolygonShape.h>
#include <simulation/cvWorld.h>
#include <collision/GJK.h>
#include <DebugDraw.h>

using namespace std;

WorldIntegration::WorldIntegration()
{
    cvWorldCInfo cInfo;
    m_world = new cvWorld(cInfo);

    m_shape = shared_ptr<cvPolygonShape> (
            cvPolygonShape::createBox(cvVec2f(2.0f, 2.0f), 0.05f));
    cvBodyCInfo bodyInfo;
    bodyInfo.m_initTransform.m_Translation = cvVec2f(5.0f, 0.0f);
    bodyInfo.m_initTransform.m_Rotation = DEG2RAD(45);
    bodyInfo.m_mass = 1.0f;
    bodyInfo.m_shape = m_shape;

    m_Id = m_world->createBody(bodyInfo, true);
    m_world->setBodyAngularVelocity(m_Id, 1.0f);
}

void WorldIntegration::tick(cvDebugDraw& debugDraw)
{
    m_world->integrate(0.02f);
    debugDraw.DrawWorld(*m_world);

    const cvBody& body = m_world->getBody(m_Id);

    cvVec2f q(0, 0);
    GJK::cvPointQueryInput input;
    input.shape = m_shape;
    input.shapeXForm = body.getTransform();

    auto res = GJK::cvPointToConvexShape(input);
    if(res.result == GJK::GJKResult::GJK_GOOD)
        debugDraw.AddLine(res.closetPt, q, cvColorf::Yellow);
}
