#pragma once

#include <iostream>

#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Solver.h"

enum class Keys : 
	short {
	F1 = (1 << 0),
	F2 = (1 << 1),
	F3 = (1 << 2),
	F4 = (1 << 3),
	F5 = (1 << 4),
	F6 = (1 << 5),
	R = (1 << 6),
	T = (1 << 7),
	Y = (1 << 8),
	U = (1 << 9),
	I = (1 << 10),
	O = (1 << 11),
	P = (1 << 12)
};
//Holds Physics solver, abstracts all glfw/imgui graphics, input etc.
class TestApp
{
public:
	TestApp();
	int Init();
	void InitImgui();
	void LoadScene(unsigned int index);
	void UpdateLoop();
	void DrawImgui();
	void ImGuiShowRigidbodyEditor();
	void ProcessInput();
public:
	//Graphics/Window
	GLFWwindow* m_window;
	const char* m_glslVersion;
	std::string m_sceneName;
	//Physics
	Solver m_solver;
	std::vector<Handle> m_bodyHandles;
	//Timestep
	decimal m_prevTime;
	//Imgui
	bool m_showDemoWindow, m_showRigidbodyEditor, m_displayManifolds, m_drawGrid, m_renderLeafNodes;//#Bit field?
	//Input: Short =16 bits. 0-5 load scenes.. see Keys::
	short m_inputDown, m_inputPressed, m_inputHeld, m_inputReleased;
};
