#pragma once

#include <cvok2d.h>
#include <vector>

struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderPolygon;
class cvShape;
class cvBody;
class cvWorld;
class cvPolygonShape;

struct cvDebugDrawOptions
{
    bool bDrawBroadphase = false;
    bool bDrawManifoild = false;
};

struct Camera
{
	Camera()
	{
		m_center.set(0.0f, 0.0f);
		m_extent = 20.0f;
		m_zoom = 1.0f;
		m_width = 1280;
		m_height = 800;
	}

	cvVec2f ConvertScreenToWorld(const cvVec2f& screenPoint);
	cvVec2f ConvertWorldToScreen(const cvVec2f& worldPoint);
	void BuildProjectionMatrix(float* m, float zBias);

	cvVec2f m_center;
	float m_extent;
	float m_zoom;
	int m_width;
	int m_height;
};


class cvDebugDraw
{
    struct dbPoly
    {
        const cvVec2f* m_pt;
        const int numPt;
        const cvTransform m_transform;
    };
public:
	cvDebugDraw();
    ~cvDebugDraw();

	void AddPoint(const cvVec2f& pos, float size, const cvColorf& color);
	void AddLine(const cvVec2f& p1, const cvVec2f& p2, const cvColorf& color);

    void AddPolygon(const std::vector<cvVec2f>& vertices, const cvTransform& trans, const cvColorf& color);
    void AddPolygon(const std::vector<cvVec2f>& vertices, const cvMat33& mat, const cvColorf& color);

    void DrawAabb(const cvAabb& aabb, const cvColorf& color);
    void DrawShape(const cvShape& shape, const cvMat33& trans, const cvColorf& color);
    void DrawPolygonShape(const cvPolygonShape& poly, cvTransform& trans, const cvColorf& color);
    void DrawBody(const cvBody& body, const cvColorf& color);


    void DrawWorld(const cvWorld& world);

	void Flush();

    cvDebugDrawOptions m_DbgDrawOpts;
private:
	GLRenderPoints* m_pointRender;
	GLRenderLines* m_lineRender;
    GLRenderPolygon* m_polyRender;
};
