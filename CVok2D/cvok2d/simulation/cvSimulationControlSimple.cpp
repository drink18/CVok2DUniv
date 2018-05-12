#include "cvSimulationControlSimple.h"
#include <collision/cvCollisionDispatch.h>
#include <shape/cvCompoundShape.h>
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

        const cvBody& bodyA = m_world->getBody(node0->m_bodyId);
        const cvBody& bodyB = m_world->getBody(node1->m_bodyId);

        if(bodyA.isStatic() && bodyB.isStatic())
            continue; //skip static pair

        const cvShape& shapeA = *bodyA.getShape();
        const cvShape& shapeB = *bodyB.getShape();

        cvMat33 matA, matB;
        bodyA.getTransform().toMat33(matA);
        bodyB.getTransform().toMat33(matB);

        if(shapeA.getShapeType() == cvShape::eCompoundShape &&
                shapeB.getShapeType() == cvShape::eCompoundShape)
        {
            continue;
        }
        else if(shapeA.getShapeType() == cvShape::eCompoundShape)
        {
            const cvCompoundShape& comp = static_cast<const cvCompoundShape&>(shapeA);
            auto& subShapes = comp.getSubshapes();
            for(auto& s : subShapes)
            {
                cvMat33 subMat;
                s.m_transform.toMat33(subMat);
                generateNPPair(simCtx, bodyA, bodyB, *s.m_shape, shapeB, subMat * matA, matB);
            }
        }
        else if(shapeB.getShapeType() == cvShape::eCompoundShape)
        {
            const cvCompoundShape& comp = static_cast<const cvCompoundShape&>(shapeB);
            auto& subShapes = comp.getSubshapes();
            for(auto& s : subShapes)
            {
                cvMat33 subMat;
                s.m_transform.toMat33(subMat);
                generateNPPair(simCtx, bodyA, bodyB, shapeA, *s.m_shape, matA, subMat * matB);
            }
        }
        else
        {
            generateNPPair(simCtx, bodyA, bodyB, shapeA, shapeB, matA,  matB);
        }
    }
}

void cvSimulationControlSimple::generateNPPair(cvSimulationContext& simCtx,
        const cvBody& bodyA, const cvBody& bodyB, 
        const cvShape& shapeA, const cvShape& shapeB,
        const cvMat33& matA, const cvMat33& matB)
{
    vector<cvNPPair>& npPairs = simCtx.m_NpPairs;
    vector<cvManifold>& manifolds = simCtx.m_Manifolds;

    npPairs.resize(npPairs.size() + 1);
    cvNPPair& np = npPairs.back();

    np.m_bodyA = bodyA.getBodyId();
    np.m_bodyB = bodyB.getBodyId();
    np.m_transA = matA;
    np.m_transB = matB;
    np.m_shapeA = const_cast<cvShape*>(&shapeA);
    np.m_shapeB = const_cast<cvShape*>(&shapeB);

    manifolds.resize(manifolds.size() + 1);
    cvManifold& manifold = manifolds.back();
    manifold.m_bodyA = np.m_bodyA;
    manifold.m_bodyB = np.m_bodyB;
    manifold.m_numPt = 0;
}

void cvSimulationControlSimple::narrowPhase(cvSimulationContext& simCtx)
{
    auto& npPairs = simCtx.m_NpPairs;
    auto& manifolds = simCtx.m_Manifolds;
    cvAssertMsg(npPairs.size() == manifolds.size(), "pair count and manifold count should match");
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
        if(body.isStatic())
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

    m_solver->solveContacts(6);
    m_solver->finishSolver(*m_world, stepInfo);
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
