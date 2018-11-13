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

vector<cvColorf> randomColors;


static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}
static ImVec4 clear_color = ImColor(114, 144, 154);
static bool singleStep = false;

static bool addPoints = false;

struct DbgDisplayControl
{
	bool showConvexHull = false;
	bool showPocket = false;
	bool showCutLine = false;
	bool highLightDonePoly = false;
	bool showOrigin = false;
};
DbgDisplayControl dbgCtrl;

typedef acd::Polygon Poly;

Loop input;
vector<Loop> polys_todo;
vector<Loop> polys_done;

static void RenderUI()
{
    ImGui_ImplGlfwGL3_NewFrame();
    // 1. show a simple window
    {
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));

		if (ImGui::Button("New Poly"))
		{
			if (!addPoints)
			{
				polys_todo.clear();
				polys_done.clear();
				polys_todo.push_back(Loop());
				addPoints = true;
			}
		}

		if (ImGui::Button("Resolve One Step"))
		{
			ResolveSingleStep();
		}

		if (ImGui::Button("Reset"))
		{
			polys_todo.clear();
			polys_done.clear();
			polys_todo.push_back(input);
		}


        ImGui::Checkbox("Show Origin only", &dbgCtrl.showOrigin);
        ImGui::Checkbox("Show Convex Hull", &dbgCtrl.showConvexHull);
        ImGui::Checkbox("Show Pockets", &dbgCtrl.showPocket);
        ImGui::Checkbox("Show Cutline", &dbgCtrl.showCutLine);
        ImGui::Checkbox("Highlight done polygones", &dbgCtrl.highLightDonePoly);
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
        g_camera.m_center.x -= delta.x;
        g_camera.m_center.y -= delta.y;
    }

    lastCursorPos.set((float)xpos, (float)ypos);
	lastCursorPos = g_camera.ConvertScreenToWorld(cvVec2f((float)xpos, (float)ypos));
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS)
		{
			if (addPoints)
			{
				polys_todo[0].AddVertex(curPos);
			}
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
			polys_todo[0].fixWinding();
			input = polys_todo[0];
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

void RenderPolyLoop(Loop &polyVerts, cvColorf color, bool solid)
{
	if (polyVerts.ptCount() == 0) return;

	if(solid)
	{ 
		auto& verts = polyVerts.getVertsArray();
		cvMat33 ident;
		g_dbgDraw->AddPolygon(verts, ident, color);
	}
	else
	{
		for (auto iter = polyVerts.begin(); iter != (polyVerts.end() - 1); iter++)
		{
			g_dbgDraw->AddArrowMid(*iter, *(iter + 1), color);
		}
	}

	if (addPoints)
	{
		g_dbgDraw->AddLine(*polyVerts.begin(), curPos, cvColorf::Yellow);
		g_dbgDraw->AddLine(*(polyVerts.end() - 1), curPos, color);
	}
	else
	{
		g_dbgDraw->AddArrowMid(*(polyVerts.end() - 1), *polyVerts.begin(), color);
	}
}

void RenderPolygon()
{
	if (dbgCtrl.showOrigin)
	{
		RenderPolyLoop(input, cvColorf::White, false);
		return;
	}

	for(auto iter = polys_todo.begin(); iter != polys_todo.end(); ++iter)
	{
		if(iter  == polys_todo.end() - 1)
			RenderPolyLoop(*iter, cvColorf::White, false);
		else
			RenderPolyLoop(*iter, cvColorf::White, false);
	}

	for(int i = 0; i < polys_done.size(); ++i)
	{
		auto& polyVerts = polys_done[i];
		cvColorf c = cvColorf::White;
		if (dbgCtrl.highLightDonePoly)
			c = randomColors[i];

		RenderPolyLoop(polyVerts, c, dbgCtrl.highLightDonePoly);
	}

	if (!polys_todo.empty())
 	{
		auto& loop = polys_todo.back();
		HullLoop hull;
		if(loop.ptCount() > 2)
			hull = _quickHull(loop);

		if (dbgCtrl.showConvexHull)
		{
			HullIdx h0(0);
			HullIdx h1(hull.pointCount() - 1);
			for (HullIdx idx = h0; idx < h1; idx++)
			{
				g_dbgDraw->AddArrowMid(loop[hull[idx]], loop[hull[idx + 1]], cvColorf::Purple);
			}

			g_dbgDraw->AddArrowMid(loop[hull[h1]], loop[hull[h0]], cvColorf::Purple);
		}


		vector<Bridge> pockets;
		if(hull.pointCount() > 0)
			pockets = _findAllPockets(hull, loop);
		if (dbgCtrl.showPocket)
		{
			for(int i = 0; i < pockets.size(); ++i)
			{
				cvColorf c = randomColors[i];
				auto& b = pockets[i];
				auto hullB0 = b.idx0;
				auto hullB1 = b.idx1;
				PolyVertIdx b0Idx = hullB0;
				PolyVertIdx b1Idx = hullB1;
				// draw bridge
				g_dbgDraw->AddArrowMid(loop[b0Idx], loop[b1Idx], c);

				PolyVertIdx prevIdx = b.notches[0];
				if (b.notches.size() == 1)
				{
					g_dbgDraw->AddArrowMid(loop[b0Idx], loop[prevIdx], c);
					g_dbgDraw->AddArrowMid(loop[b1Idx], loop[prevIdx], c);
				}
				else
				{
					PolyVertIdx curIdx = b.notches[1];
					PolyVertIdx i0 = PolyVertIdx(b0Idx > b1Idx ? b1Idx : b0Idx);
					PolyVertIdx i1 = PolyVertIdx(b0Idx < b1Idx ? b1Idx : b0Idx);
					for (auto idx = i0; idx != i1; idx = loop.nextIdx(idx))
					{
						g_dbgDraw->AddArrowMid(loop[idx], loop[loop.nextIdx(idx)], c);
					}
				}
			}
		}

		if (pockets.size() && dbgCtrl.showCutLine)
		{
			// pick best cw
			auto cw = _pickCW(loop, hull, pockets);
			g_dbgDraw->AddPoint(loop[cw.ptIndex], 20, cvColorf::Purple);
			CutLine cutLine = _findCutLine(loop, cw.ptIndex);
			g_dbgDraw->AddArrow(loop[cutLine.orgin], loop[cutLine.orgin] + cutLine.lineDir * 100, cvColorf::Cyan);
		}
	}
}


void DebugResolve()
{
	if (polys_todo.size() > 0)
	{
		for (auto polyVerts : polys_todo)
		{
			if (polyVerts.ptCount() == 0) continue;
				
			for(auto iter = polyVerts.begin(); iter != (polyVerts.end() - 1);  iter++)
			{
				g_dbgDraw->AddLine(*iter, *(iter + 1), cvColorf::Green);
			}

			if (addPoints)
			{
				g_dbgDraw->AddLine(*polyVerts.begin(), curPos, cvColorf::Yellow);
				g_dbgDraw->AddLine(*(polyVerts.end() - 1), curPos, cvColorf::Green);
			}
			else
			{
				g_dbgDraw->AddLine(*(polyVerts.end() - 1), *polyVerts.begin(), cvColorf::Green);
			}
			if (!addPoints)
			{
				//draw normals
				for (PolyVertIdx i = polyVerts.beginIdx(); i <= polyVerts.endIdx(); ++i)
				{
					cvVec2f v = polyVerts[i];
					cvVec2f nv = polyVerts[polyVerts.nextIdx(i)];
					cvVec2f mid = (v + nv) * 0.5f;
					g_dbgDraw->AddArrow(mid, mid + polyVerts.normal(i), cvColorf::Purple);
				}
			}
		}

	}
}

void ResolveSingleStep()
{
	vector<Loop> result;
	if (polys_todo.size() == 0)
		return;

	Loop loop = polys_todo.back();
	polys_todo.pop_back();

	vector<Loop> decomped = _resolveLoop(loop);
	
	if (decomped.size() == 1)
	{
		polys_done.push_back(decomped[0]);
	}
	else
	{
		for (auto& dloop : decomped)
			polys_todo.push_back(dloop);
	}
}

void ResolveSingleStepWithDebugDraw()
{
	if (!addPoints && polys_todo.size() > 0)
	{
		auto& polyVerts = polys_todo[0];
		Loop polyLoop(polyVerts);
		auto hullLoop = _quickHull(polyLoop);
		if (hullLoop.pointCount() > 0)
		{
			auto prev = hullLoop[HullIdx(0)];
			for (int i = 1; i < hullLoop.pointCount(); ++i)
			{
				auto cur = hullLoop[HullIdx(i)];
				g_dbgDraw->AddLine(polyLoop[cur], polyLoop[prev], cvColorf::Red);
				prev = cur;
			}
			g_dbgDraw->AddLine( polyLoop[hullLoop[HullIdx(0)]], polyLoop[prev], cvColorf::Red);
		}

		auto bridges = _findAllPockets(hullLoop, polyLoop);
		for(auto& b : bridges)
		{
			auto hullB0 = b.idx0;
			auto hullB1 = b.idx1;
			PolyVertIdx b0Idx = hullB0;
			PolyVertIdx b1Idx = hullB1;
			// draw bridge
			g_dbgDraw->AddLine(polyLoop[b0Idx], polyLoop[b1Idx], cvColorf::Yellow);

			auto prevIdx = b.notches[0];
			g_dbgDraw->AddLine(polyLoop[b0Idx], polyLoop[prevIdx], cvColorf::Blue);
			for (int j = 1; j < b.notches.size(); j++)
			{
				PolyVertIdx curIdx = b.notches[j];
				g_dbgDraw->AddLine(polyLoop[curIdx], polyLoop[prevIdx], cvColorf::Blue);
				prevIdx = curIdx;
			}
			g_dbgDraw->AddLine(polyLoop[prevIdx], polyLoop[b1Idx], cvColorf::Blue);
		}

		if (bridges.size())
		{
			// pick best cw
			auto cw = _pickCW(polyLoop, hullLoop, bridges);
			g_dbgDraw->AddPoint(polyLoop[cw.ptIndex], 20, cvColorf::Purple);


			// find best cut point
			PolyVertIdx cutPointIdx = _findBestCutPt(polyLoop, hullLoop, bridges, cw);
			g_dbgDraw->AddLine(polyLoop[cutPointIdx], polyLoop[cw.ptIndex], cvColorf::Green);
			g_dbgDraw->AddPoint(polyLoop[cutPointIdx], 20, cvColorf::Green);
		}
	}
}

int main(int, char**)
{
	// fill random color list
	for (int i = 0; i < 1024; ++i)
	{
		randomColors.push_back(cvColorf::randomColor());
	}

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

		glfwGetFramebufferSize(window, &g_camera.m_width, &g_camera.m_height);
		glViewport(0, 0, g_camera.m_width, g_camera.m_height);

		RenderPolygon();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		{
			pdbgDraw->Flush();
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

