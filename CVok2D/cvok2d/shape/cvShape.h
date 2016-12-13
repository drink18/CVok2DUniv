#pragma once

class cvShape
{
public:
    enum ShapeType
    {
        eCircle,
        ePolygon,
        eShapeType_Count
    };
    virtual ShapeType getShapeType() const = 0;
    virtual ~cvShape() {}
};
