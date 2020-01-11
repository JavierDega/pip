
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
		ImGui::Text("Press 0-5 to load scenes");
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
	ImGui::SetNextWindowSize(ImVec2(430, 450), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Rigidbody Editor", &m_showRigidbodyEditor))
	{
		ImGui::End();
		return;
	}

	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
	ImGui::Columns(2);
	ImGui::Separator();

	/*struct funcs
	{
		static void ShowDummyObject(const char* prefix, int uid)
		{
			ImGui::PushID(uid);                      // Use object uid as identifier. Most commonly you could also use the object pointer as a base ID.
			ImGui::AlignTextToFramePadding();  // Text and Tree nodes are less high than regular widgets, here we add vertical spacing to make the tree lines equal high.
			bool node_open = ImGui::TreeNode("Object", "%s_%u", prefix, uid);
			ImGui::NextColumn();
			ImGui::AlignTextToFramePadding();
			ImGui::Text("my sailor is rich");
			ImGui::NextColumn();
			if (node_open)
			{
				static float dummy_members[8] = { 0.0f,0.0f,1.0f,3.1416f,100.0f,999.0f };
				for (int i = 0; i < 8; i++)
				{
					ImGui::PushID(i); // Use field index as identifier.
					if (i < 2)
					{
						ShowDummyObject("Child", 424242);
					}
					else
					{
						// Here we use a TreeNode to highlight on hover (we could use e.g. Selectable as well)
						ImGui::AlignTextToFramePadding();
						ImGui::TreeNodeEx("Field", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Bullet, "Field_%d", i);
						ImGui::NextColumn();
						ImGui::SetNextItemWidth(-1);
						if (i >= 5)
							ImGui::InputFloat("##value", &dummy_members[i], 1.0f);
						else
							ImGui::DragFloat("##value", &dummy_members[i], 0.01f);
						ImGui::NextColumn();
					}
					ImGui::PopID();
				}
				ImGui::TreePop();
			}
			ImGui::PopID();
		}
	};

	// Iterate dummy objects with dummy members (all the same data)
	for (int obj_i = 0; obj_i < 3; obj_i++)
		funcs::ShowDummyObject("Object", obj_i);
		*/

	for (int i = 0; i < m_solver.m_rigidbodies.size(); i++) {
		//Info
		Rigidbody * rb = m_solver.m_rigidbodies[i];
		ImGui::PushID(i);                      // Use object uid as identifier. Most commonly you could also use the object pointer as a base ID.
		ImGui::AlignTextToFramePadding();  // Text and Tree nodes are less high than regular widgets, here we add vertical spacing to make the tree lines equal high.
		bool nodeOpen = ImGui::TreeNode("Object", "%s_%u", "Object", i);
		ImGui::NextColumn();
		ImGui::AlignTextToFramePadding();
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

		/* Tut char buffering with formatting
		char buffer[100];
		int cx;

		cx = snprintf(buffer, 100, "The half of %d is %d", 60, 60 / 2);

		if (cx >= 0 && cx < 100)      // check returned value

			snprintf(buffer + cx, 100 - cx, ", and the half of that is %d.", 60 / 2 / 2);

		puts(buffer);
		*/

		ImGui::Text(objDesc2);
		ImGui::NextColumn();
		if (nodeOpen)
		{
			//Show rotation, velocity, angular velocity
			
			ImGui::AlignTextToFramePadding();
			ImGui::TreeNodeEx("Rotation", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Bullet, "Rotation");
			ImGui::NextColumn();
			ImGui::SetNextItemWidth(-1);
			char rotation[50];
			snprintf(rotation, 50, "Rad(%f), Deg(%f)", rb->m_rotation, rb->m_rotation*RAD2DEG);
			ImGui::Text(rotation);
			//ImGui::InputFloat("##value",
			ImGui::NextColumn();
			ImGui::TreePop();

			/*ImGui::AlignTextToFramePadding();
			ImGui::TreeNodeEx("Velocity", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Bullet, "Velocity");
			ImGui::NextColumn();
			ImGui::SetNextItemWidth(-1);
			ImGui::InputFloat2("Velocity")
			ImGui::InputFloat("##value", &dummy_members[j], 1.0f)*/

			static float dummy_members[8] = { 0.0f,0.0f,1.0f,3.1416f,100.0f,999.0f };
			for (int j = 0; j < 8; j++)
			{
				ImGui::PushID(j); // Use field index as identifier.
				if (j < 2)
				{
					//ShowDummyObject("Child", 424242);
				}
				else
				{
					// Here we use a TreeNode to highlight on hover (we could use e.g. Selectable as well)
					ImGui::AlignTextToFramePadding();
					ImGui::TreeNodeEx("Field", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Bullet, "Field_%d", j);
					ImGui::NextColumn();
					ImGui::SetNextItemWidth(-1);
					if (j >= 5)
						ImGui::InputFloat("##value", &dummy_members[j], 1.0f);
					else
						ImGui::DragFloat("##value", &dummy_members[j], 0.01f);
					ImGui::NextColumn();
				}
				ImGui::PopID();
			}
			ImGui::TreePop();
		}
		ImGui::PopID();
	}
	ImGui::Columns(1);
	ImGui::Separator();
	ImGui::PopStyleVar();
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
		m_solver.AddBody(new Capsule( Vector2(0, 10), 45 * DEG2RAD, Vector2(), 0.0f, Vector2(), 1.0f, false, .2f, 1.0f));
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
	short zero = glfwGetKey(m_window, GLFW_KEY_0);
	short one = glfwGetKey(m_window, GLFW_KEY_1);
	short two = glfwGetKey(m_window, GLFW_KEY_2);
	short three = glfwGetKey(m_window, GLFW_KEY_3);
	short four = glfwGetKey(m_window, GLFW_KEY_4);
	short five = glfwGetKey(m_window, GLFW_KEY_5);
	short r = glfwGetKey(m_window, GLFW_KEY_R);
	short t = glfwGetKey(m_window, GLFW_KEY_T);
	short inputDownNew = (zero << 0) | (one << 1) | (two << 2) | (three << 3) | (four << 4) | (five << 5) | (r << 6) | (t << 7);
	//AND with m_inputDown to get m_inputHeld
	m_inputHeld = m_inputDown & inputDownNew;
	m_inputPressed = ~m_inputDown & inputDownNew;
	//etc, then assign new inputDown
	m_inputReleased = m_inputDown & ~inputDownNew;
	m_inputDown = inputDownNew;

	//React to input
	if (m_inputPressed & KEY_0)LoadScene(0);
	if (m_inputPressed & KEY_1)LoadScene(1);
	if (m_inputPressed & KEY_R)m_solver.m_stepMode ^= 1;
	if (m_inputPressed & KEY_T)m_solver.m_stepOnce = m_solver.m_stepMode & true;
}
