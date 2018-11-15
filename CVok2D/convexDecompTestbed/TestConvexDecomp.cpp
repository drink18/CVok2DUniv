#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>

#include <GL/gl3w.h>
#include <imgui.h>
#include "imgui_gl3/imgui_impl_glfw_gl3.h"
#include <GLFW/glfw3.h>
#include <algorithm>

#include "DebugDraw.h"
#include "acd.h"
#include <signal.h>
#include <chrono>

using namespace std;
using namespace acd;
using namespace chrono;

//forward decl
void ResolveSingleStep();

Camera g_camera;
cvDebugDraw* g_dbgDraw = nullptr;
cvDebugDraw* GetDebugDraw() { return g_dbgDraw; }

static bool rightBtnDown = false;
static cvVec2f curPos;
static cvVec2f dragStartPos;
static cvVec2f lastCursorPos;
static int dumpCount = 0;

vector<cvColorf> g_randomColors;

static ImVec4 clear_color = ImColor(0.2f, 0.2f, 0.2f);
static bool singleStep = false;

static bool g_addPoints = false;

struct DbgDisplayControl
{
	bool showConvexHull = false;
	bool showPocket = false;
	bool showCutLine = false;
	bool showOrigin = false;
	bool hideDonePolygon = false;
};
DbgDisplayControl dbgCtrl;

typedef acd::Polygon Poly;

Loop g_inputLoop;
vector<Poly> g_inputs;
vector<Poly> g_polys_todo;
vector<Poly> g_polys_done;


void LoadPolygonsFromFile(const char* filename)
{
	string inputFile(filename);
	ifstream fs(inputFile);
	if (fs.is_open())
	{
		g_inputs = readPolygon(fs);
		for (auto& p : g_inputs)
			g_polys_todo.push_back(p);
		g_polys_done.clear();
	}
}
static void RenderUI()
{
    ImGui_ImplGlfwGL3_NewFrame();
    // 1. show a simple window
    {
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));

		if (ImGui::Button("New Poly"))
		{
			if (!g_addPoints)
			{
				//polys_todo.push_back(Loop());
				g_addPoints = true;
			}
		}

		ImGui::Spacing();
		if (ImGui::Button("Clear All"))
		{
			if (!g_addPoints)
			{
				g_inputs.clear();
				g_inputs.resize(1);
				g_polys_done.clear();
				g_polys_todo.clear();
			}
		}

		ImGui::Spacing();
		if (ImGui::Button("Resolve One Step"))
		{
			ResolveSingleStep();
		}

		ImGui::Spacing();
		if (ImGui::Button("Resolve"))
		{
			for (auto& p : g_polys_todo)
			{
				auto res = acd::_resolveLoop_All(p);
				for (auto& donePoly : res)
					g_polys_done.push_back(donePoly);
			}
		}

		ImGui::Spacing();
		if (ImGui::Button("Reset Progress"))
		{
			g_polys_todo.clear();
			g_polys_done.clear();
			g_polys_todo = g_inputs;
		}

		ImGui::Separator();
        ImGui::Checkbox("Show Origin only", &dbgCtrl.showOrigin);
        ImGui::Checkbox("Show Convex Hull", &dbgCtrl.showConvexHull);
        ImGui::Checkbox("Show Pockets", &dbgCtrl.showPocket);
        ImGui::Checkbox("Show Cutline", &dbgCtrl.showCutLine);
        ImGui::Checkbox("Hide Done Polygons", &dbgCtrl.hideDonePolygon);

		ImGui::Separator();
		ImGui::Spacing();
		if (ImGui::Button("Dump Input Poly"))
		{
			if (g_inputs.size() > 0)
			{
				char fn[1024];
				snprintf(fn, 1024, "graph%d.txt", rand());
				string infile(fn);
				string reffile = "out_" + infile;
				ofstream outFs(fn, ios::out);
				writePolygonListInfo(outFs, g_inputs);
				vector<Poly> refs = acd::_resolvePolyList(g_inputs);
				ofstream refFs(reffile, ios::out);
				writePolygonListInfo(refFs, refs);
				dumpCount++;
			}
		}

        ImGui::PopStyleColor();
    }
    ImGui::Render();
}

static void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error %d: %s\n", error, description);
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
			if (g_addPoints)
			{
				g_inputLoop.AddVertex(curPos);
			}
		}
	}
    if (button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        if (action == GLFW_PRESS)
        {
            rightBtnDown = true;
            dragStartPos = curPos;

			if (g_addPoints)
			{
				g_addPoints = false;
			}
			g_inputLoop.fixWinding();
			g_inputs[0].addLoop(g_inputLoop);
			g_inputs[0].initializeAll();
			g_polys_todo.clear();
			g_polys_todo.push_back(g_inputs[0]);
			g_inputLoop = Loop();
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

void drop_callback(GLFWwindow* window, int count, const char** paths)
{
	LoadPolygonsFromFile(paths[0]);
}

void RenderPolyLoop(const Loop& polyVerts, cvColorf color, bool solid, bool close)
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
		for (auto iter = polyVerts.cbegin(); iter != (polyVerts.cend() - 1); iter++)
		{
			g_dbgDraw->AddArrowMid(*iter, *(iter + 1), color);
		}

		if (close)
			g_dbgDraw->AddArrowMid(*(polyVerts.cend() - 1), *(polyVerts.cbegin()), color);
	}

}

void RenderPoly(const Poly& poly, cvColorf color, bool solid)
{
	for(auto iter = poly.cbegin(); iter != poly.cend(); ++iter)
	{
		RenderPolyLoop(*iter, color, solid && !poly.hasHole(), true);
	}
}

void RenderPolygon()
{
	if (dbgCtrl.showOrigin)
	{
		for(auto& p : g_inputs)
			RenderPoly(p, cvColorf::White, false);
		return;
	}

	// render current working loop
	{
		const bool closed = false;
		RenderPolyLoop(g_inputLoop, cvColorf::White, false, closed);
		auto& polyVerts = g_inputLoop.getVertsArray();
		if (polyVerts.size() > 0)
		{
			if (g_addPoints)
			{
				g_dbgDraw->AddLine(*polyVerts.begin(), curPos, cvColorf::Yellow);
				g_dbgDraw->AddLine(*(polyVerts.end() - 1), curPos, cvColorf::White);
			}
			else
			{
				g_dbgDraw->AddArrowMid(*(polyVerts.end() - 1), *polyVerts.begin(), cvColorf::White);
			}
		}
	}

	for(auto iter = g_polys_todo.begin(); iter != g_polys_todo.end(); ++iter)
	{
		size_t idx = iter - g_polys_todo.begin();
		RenderPoly(*iter, g_randomColors[idx], false);
	}

	if (!dbgCtrl.hideDonePolygon)
	{
		for (int i = 0; i < g_polys_done.size(); ++i)
		{
			auto& polyVerts = g_polys_done[i];
			RenderPoly(polyVerts, g_randomColors[i], true);
		}
	}

	if (!g_polys_todo.empty())
 	{
		Poly& polygon = g_polys_todo.back();
		auto& loop = polygon.outterLoop();
		HullLoop hull;
		if(loop.ptCount() > 2)
			hull = quickHull(polygon);

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


		vector<Pocket> pockets;
		if(hull.pointCount() > 0)
			pockets = findAllPockets(hull, loop);
		if (dbgCtrl.showPocket)
		{
			for(int i = 0; i < pockets.size(); ++i)
			{
				cvColorf c = g_randomColors[i];
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
			auto cw = pickCW(polygon, hull, pockets);
			g_dbgDraw->AddPoint(polygon.loops[cw.loopIndex][cw.ptIndex], 20, cvColorf::Purple);
			CutLine cutLine = findCutLine(polygon, cw);
			g_dbgDraw->AddArrow(cutLine.originPt, cutLine.originPt + cutLine.lineDir * 100, cvColorf::Cyan);
		}
	}
}

void ResolveSingleStep()
{
	vector<Loop> result;
	if (g_polys_todo.size() == 0)
		return;

	Poly poly = g_polys_todo.back();
	g_polys_todo.pop_back();

	vector<Poly> decomped = _resolveLoop_OneStep(poly);

	if (decomped.size() == 1 && poly.loops.size() == decomped.cbegin()->loops.size())
	{
		g_polys_done.push_back(decomped[0]);
	}
	else
	{
		for (auto& dloop : decomped)
			g_polys_todo.push_back(dloop);
	}
}
void AbortHandler(int signal)
{
	char fileName[1024];
	snprintf(fileName, 1024, "errorInput%d.txt", rand());
	fstream file(fileName, fstream::out);
	writePolygonListInfo(file, g_inputs);
	file.close();
	fprintf(stderr, "something went wrong. Input written to %s\n", fileName);
}

int main(int narg, char** args)
{
	srand((int)system_clock::now().time_since_epoch().count());

	signal(SIGABRT, AbortHandler);
	signal(SIGSEGV, AbortHandler);

	g_inputs.resize(1);

	// fill random color list
	for (int i = 0; i < 1024; ++i)
	{
		g_randomColors.push_back(cvColorf::randomColor());
	}
	// load input file
	if (narg > 1)
	{
		LoadPolygonsFromFile(args[1]);
	}

	//cvAssert(false);
	SetErrorMode(SEM_NOGPFAULTERRORBOX);
	
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
	glfwSetDropCallback(window, drop_callback);

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

