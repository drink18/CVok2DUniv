#include "DebugDraw.h"


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <vector>

#include <cmath>

#include <core/cvColor.h>
#include <core/cvMath.h>
#include <shape/cvShape.h>
#include <shape/cvPolygonShape.h>
#include <shape/cvCircle.h>
#include <simulation/cvBody.h>
#include <simulation/cvWorld.h>


#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

extern Camera g_camera;

//
cvVec2f Camera::ConvertScreenToWorld(const cvVec2f& ps)
{
	float w = float(m_width);
	float h = float(m_height);
	float u = ps.x / w;
	float v = (h - ps.y) / h;

	float ratio = w / h;
	cvVec2f extents(ratio * m_extent, m_extent);
	extents.setScale(m_zoom);

	cvVec2f lower; lower.setSub(m_center, extents);
	cvVec2f upper; upper.setAdd(m_center, extents);

	cvVec2f pw;
	pw.x = (1.0f - u) * lower.x + u * upper.x;
	pw.y = (1.0f - v) * lower.y + v * upper.y;
	return pw;
}

//
cvVec2f Camera::ConvertWorldToScreen(const cvVec2f& pw)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
	cvVec2f extents(ratio * m_extent, m_extent);
	extents.setScale(m_zoom);

	cvVec2f lower; lower.setSub(m_center, extents);
	cvVec2f upper; upper.setAdd(m_center, extents);

	float u = (pw.x - lower.x) / (upper.x - lower.x);
	float v = (pw.y - lower.y) / (upper.y - lower.y);

	cvVec2f ps;
	ps.x = u * w;
	ps.y = (1.0f - v) * h;
	return ps;
}

void Camera::BuildProjectionMatrix(float* m, float zBias)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
    cvVec2f extents(ratio * m_extent, m_extent);
	extents.setScale(m_zoom);

	cvVec2f lower; lower.setSub(m_center, extents);
	cvVec2f upper; upper.setAdd(m_center, extents);

	m[0] = 2.0f / (upper.x - lower.x);
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / (upper.y - lower.y);
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = 1.0f;
	m[11] = 0.0f;

	m[12] = - (upper.x + lower.x) / (upper.x - lower.x);
	m[13] = - (upper.y + lower.y) / (upper.y - lower.y);
	m[14] = zBias;
	m[15] = 1.0f;
}

// Prints shader compilation errors
static void sPrintLog(GLuint object)
{
	GLint log_length = 0;
	if (glIsShader(object))
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else if (glIsProgram(object))
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else
	{
		fprintf(stderr, "printlog: Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(object))
		glGetShaderInfoLog(object, log_length, NULL, log);
	else if (glIsProgram(object))
		glGetProgramInfoLog(object, log_length, NULL, log);

	fprintf(stderr, "%s", log);
	free(log);
}

static void sCheckGLError()
{
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR)
	{
		fprintf(stderr, "OpenGL error = %d\n", errCode);
		assert(false);
	}
}

//
static GLuint sCreateShaderFromString(const char* source, GLenum type)
{
	GLuint res = glCreateShader(type);
	const char* sources[] = { source };
	glShaderSource(res, 1, sources, NULL);
	glCompileShader(res); 
	GLint compile_ok = GL_FALSE;
	glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE)
	{
		fprintf(stderr, "Error compiling shader of type %d!\n", type);
		sPrintLog(res);
		glDeleteShader(res);
		return 0;
	}

	return res;
}

// 
static GLuint sCreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
	GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
	assert(vsId != 0 && fsId != 0);

	GLuint programId = glCreateProgram();
	glAttachShader(programId, vsId);
	glAttachShader(programId, fsId);
	glBindFragDataLocation(programId, 0, "color");
	glLinkProgram(programId);

	glDeleteShader(vsId);
	glDeleteShader(fsId);

	GLint status = GL_FALSE;
	glGetProgramiv(programId, GL_LINK_STATUS, &status);
	assert(status != GL_FALSE);

	return programId;
}

//
struct GLRenderPoints
{
	void Create()
	{
		const char* vs = \
			"#version 400\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"layout(location = 2) in float v_size;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"   gl_PointSize = v_size;\n"
			"}\n";

		const char* fs = \
			"#version 400\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;
		m_sizeAttribute = 2;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(3, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);
		glEnableVertexAttribArray(m_sizeAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cvVec2f) * e_maxVertices, nullptr, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cvColorf) * e_maxVertices, nullptr, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * e_maxVertices, nullptr, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const cvVec2f& v, const cvColorf& c, float size)
	{
		m_vertices.push_back(v);
		m_sizes.push_back(size);
		m_colors.push_back(c);
	}

	void Flush()
	{
		if (m_vertices.size() == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.0f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_vertices.size() * sizeof(cvVec2f), &m_vertices[0]);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_colors.size() * sizeof(cvColorf), &m_colors[0]);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_sizes.size() * sizeof(float), &m_sizes[0]);

		glEnable(GL_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_vertices.size());
		glDisable(GL_PROGRAM_POINT_SIZE);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_vertices.clear();
		m_colors.clear();
		m_sizes.clear();
	}

	enum { e_maxVertices = 512 };
	std::vector<cvVec2f> m_vertices;
	std::vector<cvColorf> m_colors;
	std::vector<float> m_sizes;

	GLuint m_vaoId;
	GLuint m_vboIds[3];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
	GLint m_sizeAttribute;
};


struct GLRenderLines
{
	void Create()
	{
		const char* vs = \
			"#version 400\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 400\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// generate 
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);
		
		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// vertex buffer 
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cvVec2f) * e_maxVertices, nullptr, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, 0);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cvColorf) * e_maxVertices, nullptr, GL_DYNAMIC_DRAW);

		sCheckGLError();

		//clean up
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const cvVec2f& v1, const cvVec2f& v2, const cvColorf& c1, const cvColorf& c2)
	{
		if (m_vertices.size() == e_maxVertices)
			Flush();

		m_vertices.push_back(v1);
		m_vertices.push_back(v2);

		m_colors.push_back(c1);
		m_colors.push_back(c2);
	}

	void Flush()
	{
		if (m_vertices.size() == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.1f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_vertices.size() * sizeof(cvVec2f), &m_vertices[0]);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_colors.size() * sizeof(cvColorf), &m_colors[0]);

		glDrawArrays(GL_LINES, 0, (GLsizei) m_vertices.size());

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_vertices.clear();
		m_colors.clear();
	}

	enum { e_maxVertices = 2 * 512 };
	std::vector<cvVec2f> m_vertices;

	std::vector<cvColorf> m_colors;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

cvDebugDraw::cvDebugDraw()
{
	m_pointRender = new GLRenderPoints();
	m_pointRender->Create();

	m_lineRender = new GLRenderLines();
	m_lineRender->Create();
}

void cvDebugDraw::AddPoint(const cvVec2f& pos, float size, const cvColorf& color)
{
	m_pointRender->Vertex(pos, color, size);
}

void cvDebugDraw::AddLine(const cvVec2f& p1, const cvVec2f& p2, const cvColorf& color)
{
	m_lineRender->Vertex(p1, p2, color, color);
}

void cvDebugDraw::Flush()
{
	m_pointRender->Flush();
	m_lineRender->Flush();
}

void cvDebugDraw::DrawShape(const cvShape& shape, const cvTransform& trans, const cvColorf& color)
{
    cvMat33 mat;
    mat.setTranslation(trans.m_Translation);
    mat.setRotation(trans.m_Rotation);
    switch(shape.getShapeType())
    {
        case cvShape::ePolygon:
            {
                const cvPolygonShape& poly = static_cast<const cvPolygonShape&>(shape);
                auto& verts = poly.getVertices();
                for(int i = 0; i < verts.size(); ++i)
                {
                    int ni = (i == verts.size() - 1) ? 0 : i + 1;

                    AddLine(mat * verts[i], mat * verts[ni], color);
                }
            }
            break;
        case cvShape::eCircle:
            {
                const cvCircle& circle = static_cast<const cvCircle&>(shape);
                const cvVec2f c = circle.getCenter();;
                const int subDiv = 16;
                float x0 = c.x;
                float y0 = circle.getRadius() + c.y;
                const float dA = 2 * CV_PI / subDiv;
                for(int i = 0; i <= subDiv; i++)
                {
                    float x = circle.getRadius() * std::sin(i * dA) + c.x;
                    float y = circle.getRadius() * std::cos(i * dA) + c.y;

                    AddLine(mat * cvVec2f(x0, y0), mat * cvVec2f(x, y), color);
                    x0 = x;
                    y0 = y;
                }
            }
            break;

        default:
            break;
    }
}

void cvDebugDraw::DrawBody(const cvBody& body, const cvColorf& color)
{
    cvShape* shape = body.getShape().get();
    DrawShape(*shape, body.getTransform(), color);
}
