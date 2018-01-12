#include "DebugDraw.h"
#include <simulation/cvWorld.h>


void cvDebugDraw::DrawWorld(const cvWorld& world)
{
    auto& bodyManager = world.getBodyManager();
    auto iter = bodyManager.getBodyIter();
    while(iter.isValid())
    {
        const cvBody& body = bodyManager.getBody(*iter);
        DrawBody(body, cvColorf::White);
        iter++;
    }
}
