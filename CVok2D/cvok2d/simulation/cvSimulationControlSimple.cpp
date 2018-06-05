#include "cvSimulationControlSimple.h"
#include <collision/cvCollisionDispatch.h>
#include <shape/cvCompoundShape.h>
#include <solver/cvPGS.h>
#include <solver/cvSolverManifold.h>
#include <world/cvMaterial.h>
#include <world/cvWorld.h>
#include <vector>

using namespace std;
cvSimulationControlSimple::cvSimulationControlSimple(cvBroadphase* bp, cvSimulationContext* simCtx,
    cvWorld* world)
    :m_bp(bp), m_world(world)
{
    m_solver = new cvPGSSolver();
}

void cvSimulationControlSimple::preCollide(cvStepInfo& stepInfo, cvSimulationContext& simCtx)
{
    cvBodyManager& bodyMgr = m_world->accessBodyManager();
    //applying gravity here?
    // for now mark all bodies dirty
    auto iter = bodyMgr.getBodyIter();
    while (iter.isValid())
    {
        auto& body = bodyMgr.getBody(*iter);
        if (body.isDynamic())
        {
            cvMotion& motion = m_world->accessBodyMotion(body.getBodyId());
            if (motion.getInvMass() != 0.0f)
            {
                motion.m_linearVel += m_world->getCInfo().m_gravity * stepInfo.m_dt;
            }
        }
        iter++;
    }
}

void cvSimulationControlSimple::updateBP(cvSimulationContext& simCtx)
{
    cvBodyManager& bodyMgr = m_world->accessBodyManager();
    // for now mark all bodies dirty
    auto iter = bodyMgr.getBodyIter();
    while (iter.isValid())
    {
        auto& body = bodyMgr.getBody(*iter);
        m_bp->markBodyDirty(body);
        iter++;
    }

    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> delPairs;
    m_bp->updateDirtyNodes(newPairs, delPairs);

    auto& npPairs = simCtx.m_NpPairs;

    // remove disjoint pairs
    {
        for (auto& p : delPairs)
        {
            auto node0 = m_bp->getBPNode(p.m_h1);
            auto node1 = m_bp->getBPNode(p.m_h2);
            cvNPPair np(node0->m_bodyId, node1->m_bodyId);

            const cvBody& bodyA = m_world->getBody(node0->m_bodyId);
            const cvBody& bodyB = m_world->getBody(node1->m_bodyId);

            if (bodyA.isStatic() && bodyB.isStatic())
                continue; //skip static pair

#if 0
            np.m_shapeKeyA = 0;
            np.m_shapeKeyB = 0;
            if(npPairs.find(np) == npPairs.end())
            {
                printf("failed to find pair, bidA=%d, bidB=%d, skA=%d, skB=%d\n",
                        np.m_bodyA.getVal(), np.m_bodyB.getVal(), np.m_shapeKeyA, np.m_shapeKeyB);
                //cvAssert(false);
            }
#endif 
            npPairs.erase(np);
        }
    }
    // add new pairs
    for (auto& p : newPairs)
    {
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);

        const cvBody& bodyA = m_world->getBody(node0->m_bodyId);
        const cvBody& bodyB = m_world->getBody(node1->m_bodyId);

        if (bodyA.isStatic() && bodyB.isStatic())
            continue; //skip static pair

        generateNPPair(simCtx, bodyA, bodyB);
    }
}

void cvSimulationControlSimple::generateNPPair(cvSimulationContext& simCtx,
    const cvBody& bodyA, const cvBody& bodyB)
{
    cvNPPair np(bodyA.getBodyId(), bodyB.getBodyId());
    simCtx.m_NpPairs.insert(np);
}

void cvSimulationControlSimple::narrowPhase(cvSimulationContext& simCtx)
{
    auto& npPairs = simCtx.m_NpPairs;
    for(auto& cnp : npPairs)
    {
        auto& np = const_cast<cvNPPair&>(cnp);
        np.EvaluateManifolds(*m_world);
    }

    // populate solve manifold
    simCtx.m_Manifolds.clear(); 
    for(auto& np : npPairs)
    {
        for(auto& m : np.m_manifolds)
        {
            cvSolverManifold sm;
            sm.m_bodyA = m.m_bodyA;
            sm.m_bodyB = m.m_bodyB;
            sm.m_normal = m.m_normal;
            sm.m_numPt = m.m_numPt;
            for(int i = 0; i < sm.m_numPt; ++i)
            {
                sm.m_points[i] = m.m_points[i];
            }
            sm.m_friction = m.m_friction;
            sm.m_restitution = m.m_restitution;
            sm.m_rollingFriction = m.m_rollingFriction;

            simCtx.m_Manifolds.push_back(sm);
        }
    }
}

void cvSimulationControlSimple::postCollide(cvSimulationContext& simCtx)
{
}

void cvSimulationControlSimple::integrate(float dt)
{
    cvBodyManager& bodyMgr = m_world->accessBodyManager();
    cvMotionManager& motionMgr = m_world->accessMotionManager();
    for (auto iter = bodyMgr.getBodyIter(); iter.isValid(); ++iter)
    {
        cvBody& body = bodyMgr.accessBody(*iter);
        if (body.isStatic())
            continue;

        cvMotion& motion = motionMgr.accessMotion(body.getMotionId());
        cvTransform& xform = motion.m_transform;

        xform.m_Rotation += motion.m_angularVel * dt;
        xform.m_Translation += motion.m_linearVel * dt;

        body.accessTransform() = xform;
    }
}

void  cvSimulationControlSimple::solve(cvSimulationContext& simCtx, const cvStepInfo& stepInfo)
{
    m_solver->setupSolverBodies(*m_world);
    m_solver->setupContactConstraints(simCtx.m_Manifolds, *m_world, simCtx, stepInfo);
    m_solver->setupFrictionConstraints(simCtx.m_Manifolds, *m_world, simCtx, stepInfo);

    m_solver->solveContacts(simCtx.m_solverIterCount);
    m_solver->finishSolver(*m_world, stepInfo);

    //retrieve cached impulse from solver manifold
    size_t mCount = 0;
    for(auto& p : simCtx.m_NpPairs)
    {
        for(auto& cm : p.m_manifolds)
        {
            auto& m = const_cast<cvManifold&>(cm);
            for(int i = 0; i < m.m_numPt; ++i)
            {
                m.m_points[i].m_normalImpl = simCtx.m_Manifolds[mCount].m_points[i].m_normalImpl;
                m.m_points[i].m_tangentImpl = simCtx.m_Manifolds[mCount].m_points[i].m_tangentImpl;
            }
            mCount++;
        }
    }
}

void cvSimulationControlSimple::simulate(cvStepInfo& stepInfo, cvSimulationContext& simCtx)
{
    updateBP(simCtx);
    preCollide(stepInfo, simCtx);
    narrowPhase(simCtx);
    postCollide(simCtx);

    solve(simCtx, stepInfo);

    integrate(stepInfo.m_dt);
}
