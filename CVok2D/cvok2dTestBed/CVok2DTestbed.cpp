// CVok2D.cpp : Defines the entry point for the console application.
//

#include <cstdio>

#include <GL/gl3w.h>
#include <imgui.h>
#include "imgui_gl3/imgui_impl_glfw_gl3.h"
#include <GLFW/glfw3.h>

#include "DebugDraw.h"
#include "testcases/TestBase.h"
#include "testcases/BasicTest.h"
#include "testcases/TestDistance.h"

Camera g_camera;
TestBase* g_currentTest = nullptr;


static void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error %d: %s\n", error, description);
}

static void RenderUI()
{
	ImVec4 clear_color = ImColor(114, 144, 154);
	ImGui_ImplGlfwGL3_NewFrame();
	// 1. show a simple window
	{
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));
		static float f = 0.0f;
		ImGui::Text("Hellow world!");
		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
		ImGui::ColorEdit3("clear color", (float*)&clear_color);
		ImGui::PopStyleColor();
	}
	ImGui::Render();
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


	cvDebugDraw* pdbgDraw = new cvDebugDraw();
	// setup imgui binding
	if (ImGui_ImplGlfwGL3_Init(window, false) == false)
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return 1;
	}

    g_currentTest = new ClosestPointTest();


	bool show_test_window = true;
	bool show_another_window = false;
	ImVec4 clear_color = ImColor(114, 144, 154);

	// main loop
	while (!glfwWindowShouldClose(window))
	{

        if(g_currentTest)
            g_currentTest->tick(*pdbgDraw);

		glfwPollEvents();

		glfwGetWindowSize(window, &g_camera.m_width, &g_camera.m_height);
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

