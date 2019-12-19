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
	void LoadScene(unsigned int index);
	void ProcessInput();
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
	bool m_showDemoWindow;
	bool m_showAnotherWindow;
	//Input: Short =16 bits. 0-5 load scenes. 6 = Step mode(Q).
	short m_inputDown;
	short m_inputPressed;
	short m_inputHeld;
	short m_inputReleased;
	bool m_keyTest = false;
};
