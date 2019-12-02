
#include "TestApp.h"

using namespace math;

TestApp::TestApp()
	:m_window(nullptr), m_glslVersion(""), m_prevTime(0)
{
}

int TestApp::Init()
{
	/* Initialize the library */
	if (!glfwInit()) return -1;

	// GL 3.0 + GLSL 130
	m_glslVersion = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

	/* Create a windowed mode window and its OpenGL context */
	m_window = glfwCreateWindow(1280, 720, "Hello World", NULL, NULL);
	if (!m_window){
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(m_window);
	//glfwSwapInterval(1); // Enable vsync
	if (glewInit() != GLEW_OK)
		std::cout << "Error!" << std::endl;
	std::cout << glGetString(GL_VERSION) << std::endl;

	//Imgui Setup
	InitImgui();
	//Graphics setup
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1, 1, -1, 1, 0.1, 100);
	glMatrixMode(GL_MODELVIEW);
	glFrontFace(GL_CCW);//Specify backface culling (Clockwise/ counter clockwise);
	//Physics setup
	m_solver.AddBody(new Circle( Vector2(7, 5), 0, Vector2(-5, 0), 0, Vector2() ));
	m_solver.AddBody(new Circle( Vector2(-7, 5), 0, Vector2(5, 0), 0, Vector2() ));
	m_solver.AddBody(new Capsule(Vector2(0, 0), 45.0f * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, 2.f, 1.0f));
	//Timestep
	m_prevTime = (decimal)glfwGetTime();
	return 0;
}

void TestApp::InitImgui()
{
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(m_window, true);
	ImGui_ImplOpenGL3_Init(m_glslVersion);
	// Our state
	m_showDemoWindow = false;
	m_showAnotherWindow = false;
	m_clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
}

void TestApp::UpdateLoop()
{
	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(m_window))
	{	
		//#ImGui
		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();

		decimal curTime = (decimal)glfwGetTime();
		decimal dt = curTime - m_prevTime;
		m_prevTime = curTime;
		/*Physics update*/
		m_solver.Update(dt);
		/* Render here */
		glClear(GL_COLOR_BUFFER_BIT);
		for (Rigidbody* rb : m_solver.m_rigidbodies) {
			//Draw
			glLoadIdentity();
			Circle* circle;
			Capsule* capsule;
			if (circle = dynamic_cast<Circle*>(rb)) {
				glScalef((float)circle->m_radius, (float)circle->m_radius, (float)circle->m_radius);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
			} else if (capsule = dynamic_cast<Capsule*>(rb)) {
				//Capsule matrix stuff
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
			}
			glBegin(GL_TRIANGLES);
			if (circle) {
				//Circle vertices from trig
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(0, 0, 0);
					glVertex3f(cos(i * PI / 180.f), sin(i * PI / 180.f), 0);
					glVertex3f(cos((i + 10.f) * PI / 180.f), sin((i + 10.f) * PI / 180.f), 0);
				}
			} 
			else if (capsule) {
				//Capsule vertices (Two circles and rectangle?)
				float offSet = (float)capsule->m_length / 2;
				float rad = (float)capsule->m_radius;
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(-offSet, 0, 0);
					glVertex3f(-offSet + rad*cos(i * PI / 180.f), rad*sin(i * PI / 180.f), 0);
					glVertex3f(-offSet + rad*cos((i + 10.f) * PI / 180.f), rad*sin((i + 10.f) * PI / 180.f), 0);
				}
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(offSet, 0, 0);
					glVertex3f(offSet + rad * cos(i * PI / 180.f), rad * sin(i * PI / 180.f), 0);
					glVertex3f(offSet + rad * cos((i + 10.f) * PI / 180.f), rad * sin((i + 10.f) * PI / 180.f), 0);
				}
				//Draw rectangle in gltriangles
				glVertex3f(-offSet, -rad, 0);
				glVertex3f(offSet, -rad, 0);
				glVertex3f(offSet, rad, 0);
				//Rect2
				glVertex3f(-offSet, -rad, 0);
				glVertex3f(offSet, rad, 0);
				glVertex3f(-offSet, rad, 0);
			}
			glEnd();
		}
		DrawImgui();
		/* Swap front and back buffers */
		glfwSwapBuffers(m_window);

		/* Poll for and process events */
		glfwPollEvents();
	}
	//Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
}

void TestApp::DrawImgui()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
	if (m_showDemoWindow)
		ImGui::ShowDemoWindow(&m_showDemoWindow);

	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	{
		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.
		//#Javier
		ImGui::Text("Solver settings: ");
		ImGui::Checkbox("Continuous Collision", &m_solver.m_continuousCollision);
		//#Javier
		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
		ImGui::Checkbox("Demo Window", &m_showDemoWindow);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Another Window", &m_showAnotherWindow);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

	// 3. Show another simple window.
	if (m_showAnotherWindow)
	{
		ImGui::Begin("Another Window", &m_showAnotherWindow);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Hello from another window!");
		if (ImGui::Button("Close Me"))
			m_showAnotherWindow = false;
		ImGui::End();
	}
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
