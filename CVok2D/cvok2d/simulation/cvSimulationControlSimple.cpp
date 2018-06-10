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

uint32_t getBodySolverId(cvWorld& world, cvBodyId id)
{
    auto& bodyA = world.getBody(id);
    cvMotionManager& motionMgr = world.accessMotionManager();
    if (bodyA.isDynamic())
    {
        auto& motion = motionMgr.getMotion(bodyA.getMotionId());
        return motion.solverBodyId;
    }
    return 0;
}

void cvSimulationControlSimple::narrowPhase(cvSimulationContext& simCtx, const cvStepInfo& stepInfo)
{
    auto& npPairs = simCtx.m_NpPairs;
    for(auto& cnp : npPairs)
    {
        auto& np = const_cast<cvNPPair&>(cnp);
        np.EvaluateManifolds(*m_world);
    }

    // prepare solver bodies
    {
        auto& solverBodies = simCtx.m_solverBodies;
        cvMotionManager& motionMgr = m_world->accessMotionManager();
        motionMgr.refreshSolverId();
        auto& mIds = motionMgr.getAllocatedIds();
        solverBodies.clear();
        solverBodies.reserve(mIds.size() + 1);
        solverBodies.resize(mIds.size() + 1);
        for (auto& id : mIds)
        {
            auto& motion = motionMgr.getMotion(id);
            int sid = motion.solverBodyId;
            auto& sbody = solverBodies[sid];
            sbody.m_transform = motion.m_transform;
            sbody.m_velocity = cvVec3f(motion.m_linearVel.x, motion.m_linearVel.y, motion.m_angularVel);
            sbody.m_posVelocity = cvVec3f(0, 0, 0);
        }
    }

    cvMotionManager& motionMgr = m_world->accessMotionManager();
    // populate constraints
    auto& contacts = simCtx.m_contactContraints;
    contacts.clear();
    size_t numPair = npPairs.size();
    for (auto& np : npPairs)
    {
        for (auto& m : np.m_manifolds)
        {
            auto& bodyA = m_world->getBody(m.m_bodyA);
            auto& bodyB = m_world->getBody(m.m_bodyB);

            auto& motionA = bodyA.isStatic() ? motionMgr.m_staticMotion : motionMgr.getMotion(bodyA.getMotionId());
            auto& motionB = bodyB.isStatic() ? motionMgr.m_staticMotion : motionMgr.getMotion(bodyB.getMotionId());

            float invMA = motionA.getInvMass();
            float invInertiaA = motionA.getInvInertia();
            float invMB = motionB.getInvMass();
            float invInertiaB = motionB.getInvInertia();


            for (int i = 0; i < m.m_numPt; ++i)
            {
                auto& pt = m.m_points[i];
                cvContactConstraint c;
                c.bodyAId = getBodySolverId(*m_world, m.m_bodyA);
                c.bodyBId = getBodySolverId(*m_world, m.m_bodyB);
                c.m_friction = m.m_friction;
                c.m_restitution = m.m_friction;
                c.m_rollingFriction = m.m_rollingFriction;


                //setup penetration 
                c.m_accumImpl = pt.m_normalImpl;
                c.m_tangentImpl = pt.m_tangentImpl;

                c.m_friction = m.m_friction;
                c.m_restitution = m.m_restitution;
                c.m_rollingFriction = m.m_rollingFriction;

                cvVec2f na = m.m_normal;
                cvVec2f nb = -m.m_normal;

                auto pa = pt.m_point + m.m_normal * pt.m_distance;
                auto pb = pt.m_point;

                cvVec2f rA = pa - bodyA.getTransform().m_Translation; //this is wrong, should use COM
                float rxnA = rA.cross(na);
                c.JA = cvVec3f(na.x, na.y, rxnA);
                c.MA = cvVec3f(invMA, invMA, invInertiaA);

                cvVec2f rB = pb - bodyB.getTransform().m_Translation;
                float rxnB = rB.cross(nb);
                c.JB = cvVec3f(nb.x, nb.y, rxnB);
                c.MB = cvVec3f(invMB, invMB, invInertiaB);

                c.bias = 0;
                const float beta = 0.8f;
                if (pt.m_distance < 0)
                {
                    const float slop = 0.01f;
                    float pen = pt.m_distance + slop;
                    c.posBias = pen * beta / stepInfo.m_dt;
                }

                // setup fricition
                cvVec2f t = m.m_normal.computePerpendicular();
                cvVec2f ta = -t;
                cvVec2f tb = t;

                c.tJA = cvVec3f(ta.x, ta.y, rA.cross(ta));
                c.tJB = cvVec3f(tb.x, tb.y, rB.cross(tb));

                c.rJA = cvVec3f(1, 1, 1);
                c.rJA.normalize();
                c.rJB = cvVec3f(-1, -1, -1);
                c.rJB.normalize();

                if(pt.m_distance < 0)
                    contacts.push_back(c);
            }
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
    m_solver->solveContacts(simCtx);
    m_solver->finishSolver(simCtx, *m_world, stepInfo);

    //retrieve cached impulse from solver manifold
    size_t mCount = 0;
    for(auto& p : simCtx.m_NpPairs)
    {
        for(auto& cm : p.m_manifolds)
        {
            auto& m = const_cast<cvManifold&>(cm);
            for(int i = 0; i < m.m_numPt; ++i)
            {
                auto& pt = m.m_points[i];
                if (pt.m_distance < 0)
                {
                    pt.m_normalImpl = simCtx.m_contactContraints[mCount].m_accumImpl;
                    pt.m_tangentImpl = simCtx.m_contactContraints[mCount].m_tangentImpl;
                    mCount++;
                }
            }
        }
    }
}

void cvSimulationControlSimple::simulate(cvStepInfo& stepInfo, cvSimulationContext& simCtx)
{
    updateBP(simCtx);
    preCollide(stepInfo, simCtx);
    narrowPhase(simCtx, stepInfo);
    postCollide(simCtx);

    solve(simCtx, stepInfo);

    integrate(stepInfo.m_dt);
}
