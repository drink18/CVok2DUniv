#include "cvPGS.h"

#include <world/cvWorld.h>
#include <collision/cvManifold.h>
#include <algorithm>
#include <iostream>
#include <simulation/cvISimulationControl.h>

using namespace std;
void cvPGSSolver::solvePenetrations(cvSimulationContext& ctx)
{
    auto& contacts = ctx.m_contactContraints;
    auto& bodies = ctx.m_solverBodies;

    for(int i = 0; i < contacts.size(); ++i)
    {
        auto& c = contacts[i];
        cvVec3f velA, velB;
        velA = bodies[c.bodyAId].m_velocity;
        velB = bodies[c.bodyBId].m_velocity;

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

        bodies[c.bodyAId].m_velocity = velA;
        bodies[c.bodyBId].m_velocity = velB;
    }
}

void cvPGSSolver::solvePositionErr(cvSimulationContext& ctx)
{
    auto& contacts = ctx.m_contactContraints;
    auto& bodies = ctx.m_solverBodies;

    for(auto& c : contacts)
    {
        cvVec3f velA, velB;
        velA = bodies[c.bodyAId].m_posVelocity;
        velB = bodies[c.bodyBId].m_posVelocity;

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

        bodies[c.bodyAId].m_posVelocity = velA;
        bodies[c.bodyBId].m_posVelocity = velB;
    }
}

void cvPGSSolver::solveFriction(cvSimulationContext& ctx)
{
    auto& contacts = ctx.m_contactContraints;
    auto& bodies = ctx.m_solverBodies;
    for(auto& c : contacts)
    {
        cvVec3f velA, velB;
        velA = bodies[c.bodyAId].m_velocity;
        velB = bodies[c.bodyBId].m_velocity;

        // eM
        float emA = c.tJA.dot(c.MA * c.tJA);
        float emB = c.tJB.dot(c.MB * c.tJB);
        float em = emA + emB;

        // relative vel
        float v = velA.dot(c.tJA) + velB.dot(c.tJB);
        float lambda = -(c.bias + v) / em;

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

        // rolling friction
        float rv = velA.dot(c.rJA) + velB.dot(c.rJB);
        float rlambda = -rv / em;
        rlambda = min(max(-c.m_accumImpl * c.m_rollingFriction, rlambda), 
                c.m_accumImpl* c.m_rollingFriction);
        velA += c.rJA * c.MA * rlambda;
        velB += c.rJB * c.MB * rlambda;

        bodies[c.bodyAId].m_velocity = velA;
        bodies[c.bodyBId].m_velocity = velB;
    }
}

void cvPGSSolver::solveContacts(cvSimulationContext& ctx)
{
    auto& contacts = ctx.m_contactContraints;
    auto& bodies = ctx.m_solverBodies;

    // warm start
    for (auto& c : contacts)
    {
        cvVec3f& velA = bodies[c.bodyAId].m_velocity;
        cvVec3f& velB = bodies[c.bodyBId].m_velocity;

        // eM
        float emA = c.JA.dot(c.MA * c.JA);
        float emB = c.JB.dot(c.MB * c.JB);
        float em = emA + emB;

        velA += c.JA * c.MA * c.m_accumImpl;
        velB += c.JB * c.MB * c.m_accumImpl;

        bodies[c.bodyAId].m_velocity = velA;
        bodies[c.bodyBId].m_velocity = velB;

        velA += c.tJA * c.MA * c.m_tangentImpl;
        velB += c.tJB * c.MB * c.m_tangentImpl;

        bodies[c.bodyAId].m_velocity = velA;
        bodies[c.bodyBId].m_velocity = velB;
    }

    for(uint32_t i = 0; i < ctx.m_solverIterCount; ++i)
        solveFriction(ctx);

    for(uint32_t i = 0; i < ctx.m_solverIterCount; ++i)
        solvePenetrations(ctx);

    for(uint32_t i = 0; i < ctx.m_solverIterCount; ++i)
        solvePositionErr(ctx);

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

void cvPGSSolver::finishSolver(cvSimulationContext& ctx, cvWorld& world, const cvStepInfo& stepInfo)
{
    auto& contacts = ctx.m_contactContraints;
    auto& bodies = ctx.m_solverBodies;

    cvMotionManager& motionMgr = world.accessMotionManager();
    auto& mIds = motionMgr.getAllocatedIds();
    for(auto& id : mIds)
    {
        auto& motion = motionMgr.accessMotion(id);
        int sid = motion.solverBodyId;
        auto& sbody = bodies[sid];
        motion.m_transform = sbody.m_transform;
        motion.m_linearVel.set(sbody.m_velocity.x, sbody.m_velocity.y);
        motion.m_angularVel = sbody.m_velocity.z;

        // applying position correction
        cvVec2f posCoorLinVel(sbody.m_posVelocity.x, sbody.m_posVelocity.y);
        motion.m_transform.m_Translation += posCoorLinVel * stepInfo.m_dt; 
        motion.m_transform.m_Rotation += sbody.m_posVelocity.z * stepInfo.m_dt;
    }
}
