#pragma once
#include "cvok2d.h"
#include "cvShape.h"
#include <vector>
#include <algorithm>

class cvCompoundShape: public cvShape
{
public:
    struct ShapeInstance
    {
        std::shared_ptr<cvShape> m_shape;
        cvTransform m_transform;
    };

    cvCompoundShape(std::vector<ShapeInstance>& shapeInstances)
        :m_shapeInstances(shapeInstances)
    {
    }

    virtual ShapeType getShapeType() const override
    {
        return cvShape::eCompoundShape;
    }

    virtual void updateAabb() override;

private:
    std::vector<ShapeInstance> m_shapeInstances;
};
