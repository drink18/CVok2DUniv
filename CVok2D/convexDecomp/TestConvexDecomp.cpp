#include <cstdio>
#include <iostream>
#include <memory>

#include <GL/gl3w.h>
#include <imgui.h>
#include "imgui_gl3/imgui_impl_glfw_gl3.h"
#include <GLFW/glfw3.h>
#include <algorithm>

#include "DebugDraw.h"
#include "acd.h"

using namespace std;
using namespace acd;

//forward decl
void ResolveSingleStep();

Camera g_camera;
cvDebugDraw* g_dbgDraw = nullptr;
cvDebugDraw* GetDebugDraw() { return g_dbgDraw; }

static bool rightBtnDown = false;
static cvVec2f curPos;
static cvVec2f dragStartPos;
static cvVec2f lastCursorPos;


static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}
static ImVec4 clear_color = ImColor(114, 144, 154);
static bool pause = false;
static bool singleStep = false;
static bool guiclick = false;

static bool addPoints = false;

typedef acd::Polygon Poly;
vector<Poly> polygones;

static void RenderUI()
{
    ImGui_ImplGlfwGL3_NewFrame();
    // 1. show a simple window
    {
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));

		if (ImGui::Button("Reset"))
		{
			if (!addPoints)
			{
				polygones.clear();
				polygones.push_back(Poly());
				addPoints = true;
			}
		}

		if (ImGui::Button("Resolve once"))
		{
			ResolveSingleStep();
			guiclick = true;
		}

        if(ImGui::Button(">>"))
            singleStep = true;

        ImGui::Checkbox("Pause", &pause);
        ImGui::PopStyleColor();
    }
    ImGui::Render();

}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    curPos.set((float)xpos, (float)ypos);
	curPos = g_camera.ConvertScreenToWorld(cvVec2f((float)xpos, (float)ypos));
    if (rightBtnDown)
    {
        cvVec2f delta = curPos - lastCursorPos;
        delta *= 0.1f;
        g_camera.m_center.x -= delta.x;
        g_camera.m_center.y += delta.y;
    }

    lastCursorPos.set((float)xpos, (float)ypos);
	lastCursorPos = g_camera.ConvertScreenToWorld(cvVec2f((float)xpos, (float)ypos));
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (guiclick)
		return;

	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS)
		{

			if(addPoints)
				polygones[0].loops[0].Vertices.push_back(curPos);
		}
	}
    if (button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        if (action == GLFW_PRESS)
        {
            rightBtnDown = true;
            dragStartPos = curPos;

			if (addPoints)
			{
				addPoints = false;
			}
        }
        else if (action == GLFW_RELEASE)
            rightBtnDown = false;
    }
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    g_camera.m_zoom += (float)yoffset;
    g_camera.m_zoom = max(g_camera.m_zoom, 0.25f);
}

void RenderPolygon()
{
	if (polygones.size() > 0)
	{
		for (auto poly : polygones)
		{
			auto& polyVerts = poly.loops[0].Vertices;
			if (polyVerts.empty()) continue;
				
			auto prev = polyVerts[0];
			for (int i = 0; i < polyVerts.size(); ++i)
			{
				auto cur = polyVerts[i];
				g_dbgDraw->AddLine(cur, prev, cvColorf::Green);
				prev = cur;
			}
			if (addPoints)
			{
				g_dbgDraw->AddLine(prev, curPos, cvColorf::Green);
				g_dbgDraw->AddLine(polyVerts[0], curPos, cvColorf::Yellow);
			}
			else
			{
				g_dbgDraw->AddLine(prev, polyVerts[0], cvColorf::Green);
			}
		}
	}
}


void ResolveSingleStep()
{
	if (!addPoints && polygones.size() > 0)
	{
		Loop l;
		auto& polyVerts = polygones[0].loops[0].Vertices;
		l.Vertices = polyVerts;
		auto hullLoop = _quickHull(l);
		auto& resPt = hullLoop.getPtIndices();
		if (resPt.size() > 0)
		{
			auto prev = resPt[0];
			for (int i = 1; i < resPt.size(); ++i)
			{
				auto cur = resPt[i];
				g_dbgDraw->AddLine(l.Vertices[cur], l.Vertices[prev], cvColorf::Red);
				prev = cur;
			}
			g_dbgDraw->AddLine(l.Vertices[resPt[0]], l.Vertices[prev], cvColorf::Red);
		}

		auto bridges = _findAllPockets(hullLoop, l);
		for(auto& b : bridges)
		{
			auto hullB0 = b.idx0;
			auto hullB1 = b.idx1;
			int b0Idx = hullLoop.polyIdx(hullB0);
			int b1Idx = hullLoop.polyIdx(hullB1);
			// draw bridge
			g_dbgDraw->AddLine(l.Vertices[b0Idx], l.Vertices[b1Idx], cvColorf::Yellow);

			auto prevIdx = b.notches[0];
			g_dbgDraw->AddLine(l.Vertices[b0Idx], l.Vertices[prevIdx], cvColorf::Blue);
			for (int j = 1; j < b.notches.size(); j++)
			{
				int curIdx = b.notches[j];
				g_dbgDraw->AddLine(l.Vertices[curIdx], l.Vertices[prevIdx], cvColorf::Blue);
				prevIdx = curIdx;
			}
			g_dbgDraw->AddLine(l.Vertices[prevIdx], l.Vertices[b1Idx], cvColorf::Blue);
		}

		// pick best cw
		auto cw = _pickCW(l, hullLoop, bridges);
		g_dbgDraw->AddPoint(l.Vertices[cw.ptIndex], 20, cvColorf::Purple);


		// find best cut point
		int cutPointIdx = _findBestCutPt(l, hullLoop, bridges, cw);
		g_dbgDraw->AddLine(l.Vertices[cutPointIdx], l.Vertices[cw.ptIndex], cvColorf::Green);
		g_dbgDraw->AddPoint(l.Vertices[cutPointIdx], 20, cvColorf::Green);
	}
}

int main(int, char**)
{
    glfwSetErrorCallback(error_callback);
    if (glfwInit() == 0)
        return 1;


#if defined(__APPLE__)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "CVok2D", nullptr, nullptr);
    if (window == NULL)
    {
        fprintf(stderr, "Failed to open GLFW mainWindow.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    gl3wInit();
    printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));


    glClearColor(clear_color.x, clear_color.y, clear_color.z, 1);

    cvDebugDraw* pdbgDraw = new cvDebugDraw();
    g_dbgDraw = pdbgDraw;
    // setup imgui binding
    if (ImGui_ImplGlfwGL3_Init(window, false) == false)
    {
        fprintf(stderr, "Could not init GUI renderer.\n");
        assert(false);
        return 1;
    }

    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);

    float lastTime = (float)glfwGetTime();
    float dt ;
    // main loop
    while (!glfwWindowShouldClose(window))
    {
        dt = (float)glfwGetTime() - lastTime;
        lastTime = (float)glfwGetTime();


        bool run = false;
        if(!pause)
        {
            run  = true;
        }
        else if(singleStep)
        {
            singleStep = false;
            run  = true;
        }

		if (run)
		{
			glfwGetFramebufferSize(window, &g_camera.m_width, &g_camera.m_height);
			glViewport(0, 0, g_camera.m_width, g_camera.m_height);

			RenderPolygon();
			ResolveSingleStep();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            {
                pdbgDraw->Flush();
            }
        }

        glfwPollEvents();

        RenderUI();

        glfwSwapBuffers(window);
    }

    // cleanup
    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();
    return 0;
}

