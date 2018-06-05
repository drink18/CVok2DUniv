#include "cvPGS.h"

#include <world/cvWorld.h>
#include <collision/cvManifold.h>
#include <algorithm>
#include <iostream>
#include <simulation/cvISimulationControl.h>



using namespace std;

void cvPGSSolver::setupSolverBodies(cvWorld& world)
{
    cvMotionManager& motionMgr = world.accessMotionManager();
    motionMgr.refreshSolverId();
    auto& mIds = motionMgr.getAllocatedIds();
    m_solverBodies.reserve(mIds.size());
    m_solverBodies.resize(mIds.size());
    for(auto& id : mIds)
    {
        auto& motion = motionMgr.getMotion(id);
        int sid = motion.solverBodyId;
        auto& sbody = m_solverBodies[sid];
        sbody.m_transform = motion.m_transform;
        sbody.m_accumImpl = 0;
        sbody.m_velocity = cvVec3f(motion.m_linearVel.x, motion.m_linearVel.y, motion.m_angularVel);
        sbody.m_posVelocity = cvVec3f(0, 0, 0) ;
    }
}

void cvPGSSolver::setupContactConstraints(const vector<cvSolverManifold> &manifolds,
                                          const cvWorld &world, const cvSimulationContext &simCtx,
                                          const cvStepInfo &stepInfo)
{
    m_ContactContraints.clear();
    for(auto& m : manifolds)
    {
        const cvBody& bodyA = world.getBody(m.m_bodyA);
        const cvBody& bodyB = world.getBody(m.m_bodyB);

        float invMA = 0, invInertiaA = 0;
        float invMB =0, invInertiaB = 0;

        int sbIdA = -1, sbIdB = -1;

        if(bodyA.getMotionId().isValid())
        {
            const cvMotion& motionA = world.getBodyMotion(m.m_bodyA);
            invMA = motionA.getInvMass();
            invInertiaA = motionA.getInvInertia();
            sbIdA = motionA.solverBodyId;
        }

        if(bodyB.getMotionId().isValid())
        {
            const cvMotion& motionB = world.getBodyMotion(m.m_bodyB);
            invMB = motionB.getInvMass();
            invInertiaB = motionB.getInvInertia();
            sbIdB = motionB.solverBodyId;
        }

        for(int i = 0; i < m.m_numPt; ++i)
        {
            cvContactConstraint constraint;
            const cvManifoldPoint& pt = m.m_points[i];

            constraint.m_manifold = const_cast<cvSolverManifold*>(&m);
            constraint.m_maniPtIdx = i;
            constraint.m_accumImpl = pt.m_normalImpl;
            //constraint.m_tangentImpl = pt.m_tangentImpl;

            constraint.m_friction = m.m_friction;
            constraint.m_restitution = m.m_restitution;
            constraint.m_rollingFriction = m.m_rollingFriction;

            cvVec2f na = m.m_normal;
            cvVec2f nb = -m.m_normal;

            auto pa = pt.m_point +  m.m_normal * pt.m_distance ;
            auto pb = pt.m_point;

            cvVec2f rA = pa - bodyA.getTransform().m_Translation; //this is wrong, should use COM
            float rxnA = rA.cross(na);
            constraint.JA = cvVec3f(na.x, na.y, rxnA);
            constraint.MA =  cvVec3f(invMA , invMA , invInertiaA);

            cvVec2f rB = pb - bodyB.getTransform().m_Translation;
            float rxnB = rB.cross(nb);
            constraint.JB =  cvVec3f(nb.x, nb.y, rxnB);
            constraint.MB =  cvVec3f(invMB , invMB, invInertiaB);

            constraint.bodyAId = sbIdA;
            constraint.bodyBId = sbIdB;

            constraint.bias = 0;
            const float beta = 0.8f;
            if(pt.m_distance < 0)
            {
                const float slop = 0.01f;
                float pen = pt.m_distance + slop;
                constraint.posBias = pen * beta / stepInfo.m_dt;
            } 

            // friction
            cvVec2f t = m.m_normal.computePerpendicular();
            cvVec2f ta = -t;
            cvVec2f tb = t;

            constraint.tJA = cvVec3f(ta.x, ta.y, rA.cross(ta));
            constraint.tJB = cvVec3f(tb.x, tb.y, rB.cross(tb));

            constraint.rJA = cvVec3f(1, 1, 1);
            constraint.rJA.normalize();
            constraint.rJB = cvVec3f(-1, -1, -1);
            constraint.rJB.normalize();

            if(pt.m_distance < 0) 
                m_ContactContraints.push_back(constraint);
        }
    }
}

void cvPGSSolver::setupFrictionConstraints( const vector<cvSolverManifold> &manifolds,
                                          const cvWorld &world, const cvSimulationContext &simCtx,
                                          const cvStepInfo &stepInfo)
{
}


void cvPGSSolver::solvePenetrations(bool warmStart)
{
    //for(auto& c : m_ContactContraints)
    for(int i = 0; i < m_ContactContraints.size(); ++i)
    {
        auto& c = m_ContactContraints[i];
        cvVec3f velA, velB;

        if(c.bodyAId >= 0)
        {
            cvSolverBody& bodyA = m_solverBodies[c.bodyAId];
            velA = bodyA.m_velocity;
        }

        if(c.bodyBId >= 0)
        {
            cvSolverBody& bodyB = m_solverBodies[c.bodyBId];
            velB = bodyB.m_velocity;
        }

        cvVec3f oldA, oldB;
        oldA = velA;
        oldB = velB;

        // eM
        float emA = c.JA.dot(c.MA * c.JA);
        float emB = c.JB.dot(c.MB * c.JB);
        float em = emA + emB;

        // relative vel
        float v = velA.dot(c.JA) + velB.dot(c.JB);

        float lambda = -(c.bias + v) / em;

        if(warmStart)
        {
            lambda = c.m_accumImpl;
            //printf("warm start with %f \n", c.m_accumImpl);
        }

        else
        {
            float oldImp = c.m_accumImpl;
            c.m_accumImpl += lambda;
            c.m_accumImpl = std::max(0.0f, c.m_accumImpl);

            lambda = c.m_accumImpl - oldImp;
        }

        velA += c.JA * c.MA * lambda;
        velB += c.JB * c.MB * lambda;

#if 0
        const float eps = 1e-5f;
        if(abs(velA.x - oldA.x) > eps
                || abs(velB.x - oldB.x) > eps)
            cvAssertMsg(false, "vel error");

        float err = velA.dot(c.JA) + velB.dot(c.JB);
        printf("c%d, re=%f, velA=%f, %f, %f, velB=%f,%f,%f\n", i, err,
			velA.x, velA.y, velA.z, velB.x, velB.y, velB.z);
#endif

        if(c.bodyAId >= 0)
            m_solverBodies[c.bodyAId].m_velocity = velA;

        if(c.bodyBId >= 0)
            m_solverBodies[c.bodyBId].m_velocity = velB;
    }
}

void cvPGSSolver::solvePositionErr()
{
    for(auto& c : m_ContactContraints)
    {
        cvVec3f velA, velB;

        if(c.bodyAId >= 0)
        {
            cvSolverBody& bodyA = m_solverBodies[c.bodyAId];
            velA = bodyA.m_posVelocity;
        }

        if(c.bodyBId >= 0)
        {
            cvSolverBody& bodyB = m_solverBodies[c.bodyBId];
            velB = bodyB.m_posVelocity;
        }

        // eM
        float emA = c.JA.dot(c.MA * c.JA);
        float emB = c.JB.dot(c.MB * c.JB);
        float em = emA + emB;

        // relative vel
        float v = velA.dot(c.JA) + velB.dot(c.JB);
        float lambda = -(c.posBias + v) / em;

        velA += c.JA * c.MA * lambda;
        velB += c.JB * c.MB * lambda;

        float err = velA.dot(c.JA) + velB.dot(c.JB);

        if(c.bodyAId >= 0)
            m_solverBodies[c.bodyAId].m_posVelocity = velA;

        if(c.bodyBId >= 0)
            m_solverBodies[c.bodyBId].m_posVelocity = velB;
    }
}

void cvPGSSolver::solveFriction(bool warmStart)
{
    for(auto& c : m_ContactContraints)
    {
        cvVec3f velA, velB;

        if(c.bodyAId >= 0)
        {
            cvSolverBody& bodyA = m_solverBodies[c.bodyAId];
            velA = bodyA.m_velocity;
        }

        if(c.bodyBId >= 0)
        {
            cvSolverBody& bodyB = m_solverBodies[c.bodyBId];
            velB = bodyB.m_velocity;
        }

        // eM
        float emA = c.tJA.dot(c.MA * c.tJA);
        float emB = c.tJB.dot(c.MB * c.tJB);
        float em = emA + emB;


        // relative vel
        float v = velA.dot(c.tJA) + velB.dot(c.tJB);
        float lambda = -(c.bias + v) / em;

        if(warmStart)
        {
            lambda = c.m_tangentImpl;
        }
        else
        {
            float miu = c.m_friction;
            float oldImp = c.m_tangentImpl;
            c.m_tangentImpl += lambda;
            lambda = c.m_tangentImpl;
            lambda = std::max(-c.m_accumImpl * miu, lambda);
            lambda = std::min(c.m_accumImpl * miu, lambda);
            c.m_tangentImpl = lambda;
            lambda = c.m_tangentImpl - oldImp;
        }

        velA += c.tJA * c.MA * lambda;
        velB += c.tJB * c.MB * lambda;

        // rolling frition
        float rv = velA.dot(c.rJA) + velB.dot(c.rJB);
        float rlambda = -rv / em;
        rlambda = min(max(-c.m_accumImpl * c.m_rollingFriction, rlambda), 
                c.m_accumImpl* c.m_rollingFriction);
        velA += c.rJA * c.MA * rlambda;
        velB += c.rJB * c.MB * rlambda;


        if(c.bodyAId >= 0)
            m_solverBodies[c.bodyAId].m_velocity = velA;

        if(c.bodyBId >= 0)
            m_solverBodies[c.bodyBId].m_velocity = velB;
    }
}

void cvPGSSolver::solveContacts(int nIter)
{
    if(m_ContactContraints.size() == 0)
        return;

    solveFriction(true);
    solvePenetrations(true);

#if 1
    for(int i = 0; i < nIter; ++i)
        solveFriction(false);
#endif

    //printf("==================BEGIN==============\n");
    for(int i = 0; i < nIter; ++i)
        solvePenetrations(false);
    //printf("==================END==============\n");

    for(int i = 0; i < nIter; ++i)
        solvePositionErr();

#if 0
    // print out error per constraint
    for(int i = 0; i < m_ContactContraints.size(); ++i)
    {
        auto& c = m_ContactContraints[i];
        cvVec3f velA, velB;

        if(c.bodyAId >= 0)
        {
            cvSolverBody& bodyA = m_solverBodies[c.bodyAId];
            velA = bodyA.m_velocity;
        }

        if(c.bodyBId >= 0)
        {
            cvSolverBody& bodyB = m_solverBodies[c.bodyBId];
            velB = bodyB.m_velocity;
        }

        float err = c.JA.dot(velA) + c.JB.dot(velB);
        printf("c%d err = %f, velA=%f, %f, %f, velB =%f, %f, %f \n", i, err, velA.x, velA.y,
                velA.z, velB.x, velB.y, velB.z);
    }
    for(int i = 0; i < m_solverBodies.size(); ++i)
    {
        auto& sbody = m_solverBodies[i];
        printf("body %d, vel=%f, %f, %f\n", i, sbody.m_velocity.x, sbody.m_velocity.y, sbody.m_velocity.z);
    }
#endif
}

void cvPGSSolver::finishSolver(cvWorld& world, const cvStepInfo &stepInfo)
{
    // write back accumulated impulse to manifold
    for (auto& c : m_ContactContraints)
    {
        auto& mp = c.m_manifold->m_points[c.m_maniPtIdx];
        mp.m_normalImpl = c.m_accumImpl;
        mp.m_tangentImpl = c.m_tangentImpl;
    }

    cvMotionManager& motionMgr = world.accessMotionManager();
    auto& mIds = motionMgr.getAllocatedIds();
    for(auto& id : mIds)
    {
        auto& motion = motionMgr.accessMotion(id);
        int sid = motion.solverBodyId;
        auto& sbody = m_solverBodies[sid];
        motion.m_transform = sbody.m_transform;
        motion.m_linearVel.set(sbody.m_velocity.x, sbody.m_velocity.y);
        motion.m_angularVel = sbody.m_velocity.z;

        // applying position correction
        cvVec2f posCoorLinVel(sbody.m_posVelocity.x, sbody.m_posVelocity.y);
        motion.m_transform.m_Translation += posCoorLinVel * stepInfo.m_dt; 
        motion.m_transform.m_Rotation += sbody.m_posVelocity.z * stepInfo.m_dt;
    }
}
