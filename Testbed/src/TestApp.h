#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

#include "Solver.h"
//Holds Physics solver, abstract all glfw/imgui graphics

enum Keys {
	KEY_0 = (1 << 0),
	KEY_1 = (1 << 1),
	KEY_2 = (1 << 2),
	KEY_3 = (1 << 3),
	KEY_4 = (1 << 4),
	KEY_5 = (1 << 5),
	KEY_R = (1 << 6),
	KEY_T = (1 << 7)
};
class TestApp
{
public:
	TestApp();

	int Init();
	void InitImgui();
	void UpdateLoop();
	void DrawImgui();
	void ImGuiShowRigidbodyEditor();
	void LoadScene(unsigned int index);
	void ProcessInput();

	//Graphics/Window
	GLFWwindow* m_window;
	const char* m_glslVersion;
	std::string m_sceneName;
	//Physics
	Solver m_solver;
	//Timestep
	decimal m_prevTime;
	//Imgui
	bool m_showDemoWindow, m_showRigidbodyEditor;
	//Input: Short =16 bits. 0-5 load scenes. 6 = Step mode(Q).
	short m_inputDown, m_inputPressed, m_inputHeld, m_inputReleased;
};
