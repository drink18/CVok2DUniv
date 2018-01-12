#pragma once

#include <cvok2d.h>

struct GLRenderPoints;
struct GLRenderLines;
class cvShape;
class cvBody;
class cvWorld;

struct Camera
{
	Camera()
	{
		m_center.set(0.0f, 0.0f);
		m_extent = 25.0f;
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
public:
	cvDebugDraw();

	void AddPoint(const cvVec2f& pos, float size, const cvColorf& color);
	void AddLine(const cvVec2f& p1, const cvVec2f& p2, const cvColorf& color);

    void DrawShape(const cvShape& shape, const cvTransform& trans, const cvColorf& color);
    void DrawBody(const cvBody& body, const cvColorf& color);

    void DrawWorld(const cvWorld& world);

	void Flush();

private:
	GLRenderPoints* m_pointRender;
	GLRenderLines* m_lineRender;
};
