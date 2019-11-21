#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <iostream>

#include "Solver.h"
//Holds Physics solver, abstract all glfw/imgui graphics
class TestApp
{
public:
	TestApp();

	int Init();
	void InitImgui();
	void UpdateLoop();
	void DrawImgui();
	//Variables
	//Graphics/Window
	GLFWwindow* m_window;
	const char* m_glslVersion;
	//Physics
	Solver m_solver;
	//Timestep
	decimal m_prevTime;
	//Imgui
	ImVec4 m_clearColor;
	bool m_showDemoWindow = false;
	bool m_showAnotherWindow = false;
};
