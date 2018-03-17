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
        world.getBodyBPAabb(body.getBodyId(), aabb);

        DrawAabb(aabb, cvColorf::Red);

        iter++;
    }
}
