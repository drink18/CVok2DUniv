#include "cvSimulationControlSimple.h"
#include <collision/cvCollisionDispatch.h>
#include <solver/cvPGS.h>
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
    while(iter.isValid())
    {
        auto& body = bodyMgr.getBody(*iter);
        if(body.getMotionId() != cvMotionId::invalid())
        {
            cvMotion& motion = m_world->accessBodyMotion(body.getBodyId());
            if(motion.getInvMass() != 0.0f)
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
    while(iter.isValid())
    {
        auto& body = bodyMgr.getBody(*iter);
        m_bp->markBodyDirty(body);
        iter++;
    }

    vector<cvBroadphase::BPPair> newPairs;
    vector<cvBroadphase::BPPair> delPairs;
    m_bp->updateDirtyNodes(newPairs, delPairs);

    for(auto& p : newPairs)
    {
        unique_ptr<cvCollisionAgent> ag(new cvCollisionAgent());
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);
        ag->m_bodyIds[0] = node0->m_bodyId;
        ag->m_bodyIds[1] = node1->m_bodyId;

        simCtx.m_colAgents.push_back(std::move(ag));
    }

    for(int i = delPairs.size() - 1; i >=0; --i)
    {
        auto& p = delPairs[i];
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);
        auto& agents = simCtx.m_colAgents;
        for(int j = agents.size() - 1; j >= 0; --j)
        {
            auto& agent = agents[j];
            if(node0->m_bodyId == agent->m_bodyIds[0] && node1->m_bodyId == agent->m_bodyIds[1])
                agents.erase(agents.cbegin() + j);
        }
    }

    vector<cvBroadphase::BPPair> allPairs;
    vector<cvNPPair>& npPairs = simCtx.m_NpPairs;
    vector<cvManifold>& manifolds = simCtx.m_Manifolds;
    npPairs.clear();
    manifolds.clear();
    m_bp->getAllPairs(allPairs);
    for(auto& p : allPairs)
    {
        auto node0 = m_bp->getBPNode(p.m_h1);
        auto node1 = m_bp->getBPNode(p.m_h2);

        npPairs.resize(npPairs.size() + 1);
        cvNPPair& np = npPairs.back();

        const cvBody& bodyA = m_world->getBody(node0->m_bodyId);
        const cvBody& bodyB = m_world->getBody(node1->m_bodyId);

        np.m_bodyA = node0->m_bodyId;
        np.m_bodyB = node1->m_bodyId;
        m_world->getBodyTransform(np.m_bodyA).toMat33(np.m_transA);
        m_world->getBodyTransform(np.m_bodyB).toMat33(np.m_transB);
        np.m_shapeA = bodyA.getShape().get();
        np.m_shapeB = bodyB.getShape().get();

        manifolds.resize(manifolds.size() + 1);
        cvManifold& manifold = manifolds.back();
        manifold.m_bodyA = np.m_bodyA;
        manifold.m_bodyB = np.m_bodyB;
        manifold.m_numPt = 0;
    }
}

void cvSimulationControlSimple::narrowPhase(cvSimulationContext& simCtx)
{
    auto& npPairs = simCtx.m_NpPairs;
    auto& manifolds = simCtx.m_Manifolds;
    for(int i = 0; i < npPairs.size(); ++i)
    {
        auto& p = npPairs[i];
        auto* shapeA = p.m_shapeA;
        auto* shapeB = p.m_shapeB;
        auto fn = cvGetCollisionFn(shapeA->getShapeType(), shapeB->getShapeType());

        fn(*shapeA, *shapeB, p.m_transA, p.m_transB, manifolds[i]);
    }
}

void cvSimulationControlSimple::postCollide(cvSimulationContext& simCtx)
{
}

void cvSimulationControlSimple::integrate(float dt)
{
	cvBodyManager& bodyMgr = m_world->accessBodyManager();
	cvMotionManager& motionMgr = m_world->accessMotionManager();
    for(auto iter = bodyMgr.getBodyIter();iter.isValid(); ++iter)
    {
        cvBody& body = bodyMgr.accessBody(*iter);
        if(body.getMotionId() == cvMotionId::invalid())
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
    m_solver->setupConstraints(simCtx.m_Manifolds, *m_world, simCtx, stepInfo);

    for(int i = 0; i < 6; ++i)
    {
        m_solver->solveContacts();
    }
    m_solver->finishSolver(*m_world);
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
