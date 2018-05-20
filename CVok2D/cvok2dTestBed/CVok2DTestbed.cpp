// CVok2D.cpp : Defines the entry point for the console application.
//

#include <cstdio>
#include <iostream>
#include <memory>

#include <GL/gl3w.h>
#include <imgui.h>
#include "imgui_gl3/imgui_impl_glfw_gl3.h"
#include <GLFW/glfw3.h>

#include "DebugDraw.h"
#include "testcases/TestBase.h"
#include "testcases/BasicTest.h"
#include "testcases/TestDistance.h"
#include "testcases/WorldIntegration.h"

using namespace std;

Camera g_camera;
unique_ptr<TestBase> g_currentTest = nullptr;
cvDebugDraw* g_dbgDraw = nullptr;
int g_currentDemoIdx = 0;

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
static void RenderUI()
{
    vector<const char*> testNames;
    for(auto& ti : GetRegisteredTests())
    {
        testNames.push_back(ti.m_name);
    }

    int oldDemoIdx = g_currentDemoIdx;
    int solverIter = g_currentTest->getSolverIteration();
    ImGui_ImplGlfwGL3_NewFrame();
    bool reset = false;
    // 1. show a simple window
    {
        ImGui::ListBox("Tests", &g_currentDemoIdx, &testNames[0], testNames.size());
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));

        if(ImGui::Button("Reset"))
            reset = true;

        // render options
        ImGui::Checkbox("Draw AABB", &g_dbgDraw->m_DbgDrawOpts.bDrawBroadphase);
        ImGui::Checkbox("Draw Manifold", &g_dbgDraw->m_DbgDrawOpts.bDrawManifoild);
        ImGui::SliderInt("Solver Iter", &solverIter, 1, 100);
        ImGui::PopStyleColor();
    }
    ImGui::Render();

    if(g_currentDemoIdx != oldDemoIdx || reset)
    {
        g_currentTest.reset(GetRegisteredTests()[g_currentDemoIdx].m_testFn());
    }
    g_currentTest->setSolverIteration(solverIter);
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    curPos.set((float)xpos, (float)ypos);
    if (rightBtnDown)
    {
        cvVec2f delta = curPos - lastCursorPos;
        delta *= 0.1f;
        g_camera.m_center.x -= delta.x;
        g_camera.m_center.y += delta.y;
    }

    lastCursorPos.set((float)xpos, (float)ypos);
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        if (action == GLFW_PRESS)
        {
            rightBtnDown = true;
            dragStartPos = curPos;
        }
        else if (action == GLFW_RELEASE)
            rightBtnDown = false;
    }
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    g_camera.m_zoom += yoffset;
    g_camera.m_zoom = std::max(g_camera.m_zoom, 0.25f);
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


    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
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

    g_currentTest.reset(GetRegisteredTests()[g_currentDemoIdx].m_testFn());

    float lastTime = glfwGetTime();
    float dt ;
    // main loop
    while (!glfwWindowShouldClose(window))
    {
        dt = glfwGetTime() - lastTime;
        lastTime = glfwGetTime();

        if (g_currentTest)
            g_currentTest->tick(*pdbgDraw, 1.0f/ 60);

        glfwPollEvents();

        glfwGetFramebufferSize(window, &g_camera.m_width, &g_camera.m_height);
        glViewport(0, 0, g_camera.m_width, g_camera.m_height);


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pdbgDraw->Flush();

        RenderUI();

        glfwSwapBuffers(window);
    }

    // cleanup
    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();
    return 0;
}

