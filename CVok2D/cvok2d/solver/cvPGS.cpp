#include "cvPGS.h"

#include <world/cvWorld.h>
#include <collision/cvManifold.h>
#include <algorithm>
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
    }
}

void cvPGSSolver::setupConstraints(const vector<cvManifold> &manifolds,
                                   const cvWorld &world, const cvSimulationContext &simCtx, const cvStepInfo &stepInfo)
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
            cvContactConstraint contact;
            const cvManifoldPoint& pt = m.m_points[i];

            cvVec2f na = m.m_normal;
            cvVec2f nb = -m.m_normal;

            auto pa = pt.m_point;
            auto pb = pt.m_point;

            cvVec2f rA = pa - bodyA.getTransform().m_Translation; //this is wrong, should use COM
            float rxnA = rA.cross(na);
            contact.JA = cvVec3f(na.x, na.y, rxnA);
            contact.MA =  cvVec3f(invMA, invMA, invInertiaA);

            cvVec2f rB = pb - bodyB.getTransform().m_Translation;
            float rxnB = rB.cross(nb);
            contact.JB =  cvVec3f(nb.x, nb.y, rxnB);
            contact.MB =  cvVec3f(invMB, invMB, invInertiaB);

            contact.manifold = m;
            contact.bodyAId = sbIdA;
            contact.bodyBId = sbIdB;

            contact.bias = pt.m_distance < 0 ? pt.m_distance * 0.4f / stepInfo.m_dt : 0;

            if(pt.m_distance < 0) 
                m_ContactContraints.push_back(contact);
        }
    }
}

static float nrelV;

void cvPGSSolver::solveContacts()
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
        float emA = c.JA.dot(c.MA * c.JA);
        float emB = c.JB.dot(c.MB * c.JB);
        float em = emA + emB;

        // relative vel
        float v = velA.dot(c.JA) + velB.dot(c.JB);

        float lambda = -(c.bias + v) / em;

        float oldImp = c.m_accumImpl;
        c.m_accumImpl += lambda;
        c.m_accumImpl = std::max(0.0f, c.m_accumImpl);

        lambda = c.m_accumImpl - oldImp;

        velA += c.JA * c.MA * lambda;
        velB += c.JB * c.MB * lambda;

        nrelV = velA.dot(c.JA) + velB.dot(c.JB);

        if(c.bodyAId >= 0)
            m_solverBodies[c.bodyAId].m_velocity = velA;

        if(c.bodyBId >= 0)
            m_solverBodies[c.bodyBId].m_velocity = velB;
    }
}

void cvPGSSolver::finishSolver(cvWorld& world)
{
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
    }
}
