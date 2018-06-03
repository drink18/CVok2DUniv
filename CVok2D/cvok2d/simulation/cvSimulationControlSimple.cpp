#include "cvSimulationControlSimple.h"
#include <collision/cvCollisionDispatch.h>
#include <shape/cvCompoundShape.h>
#include <solver/cvPGS.h>
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
            cvNPPair np;
            auto node0 = m_bp->getBPNode(p.m_h1);
            auto node1 = m_bp->getBPNode(p.m_h2);
            np.m_bodyA = node0->m_bodyId;
            np.m_bodyB = node1->m_bodyId;
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

        const cvShape& shapeA = *bodyA.getShape();
        const cvShape& shapeB = *bodyB.getShape();

        cvMat33 matA, matB;
        bodyA.getTransform().toMat33(matA);
        bodyB.getTransform().toMat33(matB);

        if (shapeA.getShapeType() == cvShape::eCompoundShape &&
            shapeB.getShapeType() == cvShape::eCompoundShape)
        {
            continue;
        }
        else if (shapeA.getShapeType() == cvShape::eCompoundShape)
        {
            const cvCompoundShape& comp = static_cast<const cvCompoundShape&>(shapeA);
            auto& subShapes = comp.getSubshapes();
            for (int i = 0; i < subShapes.size(); ++i)
            {
                generateNPPair(simCtx, bodyA, bodyB, shapeA, shapeB, i, 0);
            }
        }
        else if (shapeB.getShapeType() == cvShape::eCompoundShape)
        {
            const cvCompoundShape& comp = static_cast<const cvCompoundShape&>(shapeB);
            auto& subShapes = comp.getSubshapes();
            for (int i = 0; i < subShapes.size(); ++i)
            {
                generateNPPair(simCtx, bodyA, bodyB, shapeA, shapeB, 0, i);
            }
        }
        else
        {
            generateNPPair(simCtx, bodyA, bodyB, shapeA, shapeB, 0, 0);
        }
    }
}

void cvSimulationControlSimple::generateNPPair(cvSimulationContext& simCtx,
    const cvBody& bodyA, const cvBody& bodyB,
    const cvShape& shapeA, const cvShape& shapeB,
    cvShapeKey keyA, cvShapeKey keyB)
{
    unordered_map<cvNPPair, cvManifold>& npPairs = simCtx.m_NpPairs;

    cvNPPair np;

    np.m_bodyA = bodyA.getBodyId();
    np.m_bodyB = bodyB.getBodyId();
    np.m_shapeKeyA = keyA;
    np.m_shapeKeyB = keyB;

    cvManifold manifold;
    manifold.m_bodyA = np.m_bodyA;
    manifold.m_bodyB = np.m_bodyB;
    manifold.m_numPt = 0;
    if (bodyA.getMaterial() && bodyB.getMaterial())
    {
        manifold.m_friction = cvMaterial::CombineFirction(*bodyA.getMaterial(), *bodyB.getMaterial());
        manifold.m_rollingFriction = cvMaterial::CombineRollingFriction(*bodyA.getMaterial(), *bodyB.getMaterial());
    }
    else if (bodyA.getMaterial())
    {
        manifold.m_friction = bodyA.getMaterial()->m_friction;
        manifold.m_rollingFriction = bodyA.getMaterial()->m_rollingFriction;
    }
    else if (bodyB.getMaterial())
    {
        manifold.m_friction = bodyB.getMaterial()->m_friction;
        manifold.m_rollingFriction = bodyB.getMaterial()->m_rollingFriction;
    }
    npPairs[np] = manifold;
}

void extractSubShapeIfCompound(cvShape*& shape, cvShapeKey shapeKey, cvMat33& mat)
{
    if (shape->getShapeType() == cvShape::eCompoundShape)
    {
        cvMat33 subMat;
        auto* comp = static_cast<cvCompoundShape*>(shape);
        comp->getSubshapes()[shapeKey].m_transform.toMat33(subMat);
        mat = subMat * mat;
        shape = comp->getSubshapes()[shapeKey].m_shape.get();
    }
}

void cvSimulationControlSimple::narrowPhase(cvSimulationContext& simCtx)
{
    auto& npPairs = simCtx.m_NpPairs;
    for (auto& np : npPairs)
    {
        auto& p = np.first;
        auto& manifold = np.second;

        const cvBody& bodyA = m_world->getBody(p.m_bodyA);
        const cvBody& bodyB = m_world->getBody(p.m_bodyB);

        auto* shapeA = bodyA.getShape().get();
        auto* shapeB = bodyB.getShape().get();


        cvMat33 matA, matB;
        bodyA.getTransform().toMat33(matA);
        bodyB.getTransform().toMat33(matB);

        extractSubShapeIfCompound(shapeA, p.m_shapeKeyA, matA);
        extractSubShapeIfCompound(shapeB, p.m_shapeKeyB, matB);

        auto fn = cvGetCollisionFn(shapeA->getShapeType(), shapeB->getShapeType());

        fn(*shapeA, *shapeB, matA, matB, manifold);
    }

    auto& manifolds = simCtx.m_Manifolds;
    manifolds.clear();
    manifolds.reserve(npPairs.size());
    for (auto& np : npPairs)
        manifolds.push_back(&np.second);
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
