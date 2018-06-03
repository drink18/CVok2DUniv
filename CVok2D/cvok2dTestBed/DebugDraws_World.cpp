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

        if(m_DbgDrawOpts.bDrawBroadphase)
        {
            //broad phase
            cvAabb aabb;
            world.getBodyBPAabb(body.getBodyId(), aabb);
            DrawAabb(aabb, cvColorf::Red);
        }

        if(m_DbgDrawOpts.bDrawManifoild)
        {
            auto& simCtx = world.getSimContext();
            auto& manifolds = simCtx.m_Manifolds;
            for(auto& mp : manifolds)
            {
                auto& m = *mp;
                for(int i = 0; i < m.m_numPt; ++i)
                {
                    const cvManifoldPoint& p = m.m_points[i];
                    AddLine(p.m_point, p.m_point + m.m_normal * p.m_distance, cvColorf::Green);
                }
            }
        }


        iter++;
    }
}
