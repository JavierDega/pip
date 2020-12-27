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
	KEY_F1 = (1 << 0),
	KEY_F2 = (1 << 1),
	KEY_F3 = (1 << 2),
	KEY_F4 = (1 << 3),
	KEY_F5 = (1 << 4),
	KEY_F6 = (1 << 5),
	KEY_R = (1 << 6),
	KEY_T = (1 << 7),
	KEY_Y = (1 << 8),
	KEY_U = (1 << 9),
	KEY_I = (1 << 10),
	KEY_O = (1 << 11),
	KEY_P = (1 << 12)
};

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

	//Graphics/Window
	GLFWwindow* m_window;
	const char* m_glslVersion;
	std::string m_sceneName;
	//Physics
	Solver m_solver;
	//Timestep
	decimal m_prevTime;
	//Imgui
	bool m_showDemoWindow, m_showRigidbodyEditor, m_displayManifolds, m_drawGrid, m_renderLeafNodes;
	//Input: Short =16 bits. 0-5 load scenes. 6 = Step mode(Q).
	short m_inputDown, m_inputPressed, m_inputHeld, m_inputReleased;
};
