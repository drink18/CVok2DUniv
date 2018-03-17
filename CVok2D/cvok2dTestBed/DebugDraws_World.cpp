#include "DebugDraw.h"
#include <world/cvWorld.h>


void cvDebugDraw::DrawWorld(const cvWorld& world)
{
    auto& bodyManager = world.getBodyManager();
    auto iter = bodyManager.getBodyIter();
    while(iter.isValid())
    {
        const cvBody& body = bodyManager.getBody(*iter);
        DrawBody(body, cvColorf::White);

        //broad phase 
        cvAabb aabb;
        body.getAabb(aabb);
        cvVec2f ext = cvVec2f(0.05f, 0.05f);
        aabb.m_Max += ext;
        aabb.m_Min -= ext;

        DrawAabb(aabb, cvColorf::Red);

        iter++;
    }
}
