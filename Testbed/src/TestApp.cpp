
#include "TestApp.h"

using namespace math;

TestApp::TestApp()
	:m_window(nullptr), m_glslVersion(""), m_sceneName(""), m_prevTime(0), m_showDemoWindow(false), m_showRigidbodyEditor(false), m_inputDown(0), m_inputPressed(0), 
	m_inputHeld(0), m_inputReleased(0)
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
	m_window = glfwCreateWindow(800, 800, "Hello World", NULL, NULL);
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
	LoadScene(0);
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
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(m_window, true);
	ImGui_ImplOpenGL3_Init(m_glslVersion);
}

void TestApp::UpdateLoop()
{
	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(m_window))
	{	
		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();
		ProcessInput();
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
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glScalef((float)circle->m_radius, (float)circle->m_radius, (float)circle->m_radius);
				glBegin(GL_TRIANGLES);
				//Circle vertices from trig
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(0, 0, 0);
					glVertex3f(cos(i * DEG2RAD), sin(i * DEG2RAD), 0);
					glVertex3f(cos((i + 10.f) * DEG2RAD), sin((i + 10.f) * DEG2RAD), 0);
				}
			} 
			else if (capsule = dynamic_cast<Capsule*>(rb)) {
				//Capsule matrix stuff
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glBegin(GL_TRIANGLES);
				//Capsule vertices (Two circles and rectangle?)
				float offSet = (float)capsule->m_length / 2;
				float rad = (float)capsule->m_radius;
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(-offSet, 0, 0);
					glVertex3f(-offSet + rad * cos(i * DEG2RAD), rad * sin(i * DEG2RAD), 0);
					glVertex3f(-offSet + rad * cos((i + 10.f) * DEG2RAD), rad * sin((i + 10.f) * DEG2RAD), 0);
				}
				for (int i = 0; i < 350; i += 10) {
					glVertex3f(offSet, 0, 0);
					glVertex3f(offSet + rad * cos(i * DEG2RAD), rad * sin(i * DEG2RAD), 0);
					glVertex3f(offSet + rad * cos((i + 10.f) * DEG2RAD), rad * sin((i + 10.f) * DEG2RAD), 0);
				}
				//Draw rectangle in gltriangles
				glVertex3f(-offSet, -rad, 0);
				glVertex3f(offSet, -rad, 0);
				glVertex3f(offSet, rad, 0);
				//Tri2
				glVertex3f(-offSet, -rad, 0);
				glVertex3f(offSet, rad, 0);
				glVertex3f(-offSet, rad, 0);
			}
			glEnd();
		}
		DrawImgui();
		/* Swap front and back buffers */
		glfwSwapBuffers(m_window);
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
	if (m_showRigidbodyEditor)
		ImGuiShowRigidbodyEditor();
	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	{
		ImGui::Begin(m_sceneName.c_str());                          // Create a window and append into it.
		ImGui::Text("Press F1-F5 to load scenes");
		ImGui::Checkbox("Step mode (R)", &m_solver.m_stepMode);
		ImGui::Checkbox("Step once (T)", &m_solver.m_stepOnce);
		ImGui::Checkbox("Continuous Collision", &m_solver.m_continuousCollision);
		ImGui::Checkbox("Demo Window", &m_showDemoWindow);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Show Rigidbody Editor", &m_showRigidbodyEditor);      // Edit bools storing our window open/close state

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
//Maybe take handle to boolean
void TestApp::ImGuiShowRigidbodyEditor()
{
	ImGui::Begin("Rigidbody Editor", &m_showRigidbodyEditor);
	ImGui::Columns(2);
	ImGui::Separator();

	for (int i = 0; i < m_solver.m_rigidbodies.size(); i++) {
		Rigidbody * rb = m_solver.m_rigidbodies[i];
		//Turn to char*
		char* strId = new char[10];
		snprintf(strId, 10, "Rb%i", i);//Worth revising this
		bool nodeOpen = ImGui::TreeNode(strId, "%s_%u", "Rigidbody", i);
		ImGui::NextColumn();
		std::string objDesc;
		if (Circle * circle = dynamic_cast<Circle*>(rb)) {
			objDesc = "Circle, pos: ";
		}
		else if (Capsule * capsule = dynamic_cast<Capsule*>(rb)) {
			objDesc = "Capsule, pos: ";
		}
		//Turn to char*
		char* objDesc2 = new char[100];
		strcpy(objDesc2, objDesc.c_str());
		int firstPartLength = objDesc.length();
		snprintf(objDesc2 + firstPartLength, 100 - firstPartLength, "X(%f), Y(%f)", rb->m_position.x, rb->m_position.y);//Worth revising this

		ImGui::Text(objDesc2);
		ImGui::NextColumn();
		if (nodeOpen)
		{
			//Show rotation, velocity, angular velocity
			ImGui::Text("Rotation");
			ImGui::NextColumn();
			char rotation[50];
			snprintf(rotation, 50, "Rad(%f), Deg(%f)", rb->m_rotation, rb->m_rotation*RAD2DEG);
			ImGui::Text(rotation);
			ImGui::NextColumn();

			ImGui::Text("Velocity");
			ImGui::NextColumn();
			ImGui::DragFloat("VelX", &rb->m_velocity.x, 1.0f );//#TODO: Might not be compatible with fixedpoint mode. Create wrapper for inputfloat funcs?
			ImGui::DragFloat("VelY", &rb->m_velocity.y, 1.0f );
			ImGui::NextColumn();

			ImGui::Text("AngVel");
			ImGui::NextColumn();
			ImGui::DragFloat("Rot (Rad/S)", &rb->m_angularVelocity, 0.1f);
			ImGui::NextColumn();

			ImGui::Text("Acceleration");
			ImGui::NextColumn();
			ImGui::DragFloat("AccelX", &rb->m_acceleration.x, 1.0f);
			ImGui::DragFloat("AccelY", &rb->m_acceleration.y, 1.0f);
			ImGui::NextColumn();

			ImGui::Text("Mass");
			ImGui::NextColumn();
			ImGui::InputFloat("Mass", &rb->m_mass, 0.1f);
			ImGui::NextColumn();

			ImGui::Text("Kinematic");
			ImGui::NextColumn();
			ImGui::Checkbox("Is Kinematic?", &rb->m_isKinematic);
			ImGui::NextColumn();

			ImGui::Text("Inertia");
			ImGui::NextColumn();
			char inertia[50];
			snprintf(inertia, 50, "%f", rb->m_inertia);
			ImGui::Text(inertia);
			ImGui::NextColumn();

			ImGui::TreePop();
		}
	}
	ImGui::Columns(1);
	ImGui::Separator();
	ImGui::End();
}

void TestApp::LoadScene(unsigned int index)
{
	m_solver.m_rigidbodies.clear();
	switch (index) {
	case 0:
	{
		m_sceneName = "Scene 0: Desc here";
		//m_solver.AddBody(new Circle(Vector2(7, 5), 45 * DEG2RAD, Vector2(-5, 0), 0, Vector2()));
		m_solver.AddBody(new Circle(Vector2(-7, 5), 0, Vector2(5, 0), 0, Vector2()));
		m_solver.AddBody(new Capsule(Vector2(0, 0), 0 * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, false, 2.f, 1.0f));
	}
	break;
	case 1:
	{
		m_sceneName = "Scene 1: Desc here";
		m_solver.AddBody(new Capsule( Vector2(0, 10), 45 * DEG2RAD, Vector2(), 0.0f, Vector2(), 1.0f, false, .5f, 1.0f));
		m_solver.AddBody(new Capsule( Vector2(0, 0), 0 * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, true, 2.0f, 1.0f));
	}
	break;
	default:
		break;
	}
}

void TestApp::ProcessInput()
{
	//glfwGetKey(m_window, GLFW_KEY_0);
	short f1 = glfwGetKey(m_window, GLFW_KEY_F1);
	short f2 = glfwGetKey(m_window, GLFW_KEY_F2);
	short f3 = glfwGetKey(m_window, GLFW_KEY_F3);
	short f4 = glfwGetKey(m_window, GLFW_KEY_F4);
	short f5 = glfwGetKey(m_window, GLFW_KEY_F5);
	short r = glfwGetKey(m_window, GLFW_KEY_R);
	short t = glfwGetKey(m_window, GLFW_KEY_T);
	short inputDownNew = (f1 << 0) | (f2 << 1) | (f3 << 2) | (f4 << 3) | (f5 << 4) | (r << 5) | (t << 6);
	//AND with m_inputDown to get m_inputHeld
	m_inputHeld = m_inputDown & inputDownNew;
	m_inputPressed = ~m_inputDown & inputDownNew;
	//etc, then assign new inputDown
	m_inputReleased = m_inputDown & ~inputDownNew;
	m_inputDown = inputDownNew;

	//React to input
	if (m_inputPressed & KEY_F1)LoadScene(0);
	if (m_inputPressed & KEY_F2)LoadScene(1);
	if (m_inputPressed & KEY_R)m_solver.m_stepMode ^= 1;
	if (m_inputPressed & KEY_T)m_solver.m_stepOnce = m_solver.m_stepMode & true;
}
