#pragma once
#include "TestBase.h"
#include <vector>
#include <memory>

class cvPolygonShape;
class cvCircle;

class ClosestPointTest: public TestBase
{
public:
    ClosestPointTest();

    virtual void tick(cvDebugDraw& gdbDraw, float dt) override;
    void updateShapePair(cvDebugDraw& gdbDraw, const cvCircle& circle, 
            const cvPolygonShape& poly, const cvTransform& t1, const cvTransform& t2);


private:
    std::shared_ptr<cvPolygonShape> m_poly;
    std::shared_ptr<cvCircle> m_circle;
    cvTransform m_t1;
    cvTransform m_t2;
};
