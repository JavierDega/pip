
#include "TestApp.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

using namespace math;

TestApp::TestApp()
	:m_window(nullptr), m_glslVersion(""), m_sceneName(""), m_prevTime(0), m_showDemoWindow(false), m_showRigidbodyEditor(false), m_displayManifolds(false), m_drawGrid(false),
	m_inputDown(0), m_inputPressed(0), m_inputHeld(0), m_inputReleased(0)
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

void TestApp::LoadScene(unsigned int index)
{
	m_solver.m_rigidbodies.clear();
	switch (index) {
	case 0:
	{
		m_sceneName = "Scene 0: Desc here";
		//m_solver.AddBody(new Circle(Vector2(7, 5), 45 * DEG2RAD, Vector2(-5, 0), 0, Vector2()));
		m_solver.AddBody(new Circle(Vector2(-5, 6), 0, Vector2(5, 0), 0, Vector2()));
		m_solver.AddBody(new Capsule(Vector2(0, 0), 0 * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, false, 2.f, 1.0f));
		m_solver.AddBody(new Circle(Vector2(5, 5), 0, Vector2(-5, 0), 0, Vector2()));
	}
	break;
	case 1:
	{
		m_sceneName = "Scene 1: Desc here";
		//m_solver.AddBody(new Capsule(Vector2(0, 10), 45 * DEG2RAD, Vector2(), 0.0f, Vector2(), 1.0f, false, .5f, 1.0f));
		m_solver.AddBody(new Capsule(Vector2(0, 5), 45 * DEG2RAD, Vector2(), 0.0f, Vector2(), 10.f, false, 2.0f, 1.0f));
		m_solver.AddBody(new OrientedBox(Vector2( 0, -2), 46 * DEG2RAD, Vector2(0, 0), 0.0f, Vector2(), 100.f, false, Vector2(2, 2)));
	}
	break;
	case 2:
	{
		m_sceneName = "Scene 2: Sphere against Obb";
		m_solver.AddBody(new Circle(Vector2(0.1f, 5), 0 * DEG2RAD, Vector2(), 0.0f, Vector2(), 1.f));
		m_solver.AddBody(new OrientedBox(Vector2(3, 0), 0 * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, false));
		m_solver.AddBody(new OrientedBox(Vector2(0, 0), 45 * DEG2RAD, Vector2(), 0.0f, Vector2(), 100.f, false, Vector2(0.5f, 0.5f)));
	}
	break;
	case 3:
	{
		m_sceneName = "Scene 3: OBB collision with SAT";
		m_solver.AddBody(new OrientedBox(Vector2(-5, 5), 0 * DEG2RAD, Vector2(5, 0), 0.0f, Vector2(), 100.f, false));
		m_solver.AddBody(new OrientedBox(Vector2(5, 5), 0 * DEG2RAD, Vector2(-5, 0), 0.0f, Vector2(), 100.f, false));
	}
	default:
		break;
	}
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
		glColor3f(255, 255, 255);
		for (Rigidbody* rb : m_solver.m_rigidbodies) {
			//Draw
			glLoadIdentity();
			if ( Circle * circle = dynamic_cast<Circle*>(rb)) {
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glScalef((float)circle->m_radius, (float)circle->m_radius, (float)circle->m_radius);
				glBegin(GL_TRIANGLES);
				//Circle vertices from trig
				for (int i = 0; i < 350; i += 10) {
					//Counter clockwise
					glVertex3f(0, 0, 0);
					glVertex3f(cos(i * DEG2RAD), sin(i * DEG2RAD), 0);
					glVertex3f(cos((i + 10.f) * DEG2RAD), sin((i + 10.f) * DEG2RAD), 0);
				}
			} 
			else if ( Capsule * capsule = dynamic_cast<Capsule*>(rb)) {
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
			else if (OrientedBox* obb = dynamic_cast<OrientedBox*>(rb)) {
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glBegin(GL_TRIANGLES);
				//Rectangle made up of two triangles
				Vector2 halfExtents = obb->m_halfExtents;
				glVertex3f(-halfExtents.x, -halfExtents.y, 0);
				glVertex3f(halfExtents.x, -halfExtents.y, 0);
				glVertex3f(halfExtents.x, halfExtents.y, 0);
				//Upper tri
				glVertex3f(-halfExtents.x, -halfExtents.y, 0);
				glVertex3f(halfExtents.x, halfExtents.y, 0);
				glVertex3f(-halfExtents.x, halfExtents.y, 0);
			}
			glEnd();
		}

		//Render manifolds
		if (m_displayManifolds) {
			glColor3f(255, 0, 0);
			//Draw in red: Normal with magnitude at all contact points
			for ( Manifold& manifold : m_solver.m_currentManifolds) {
				for (int i = 0; i < manifold.numContactPoints; i++) {
					Vector2 currentPoint = manifold.contactPoints[i];
					Vector2 normalRotLeft = manifold.normal.Rotated(15 * DEG2RAD)*0.9f;
					Vector2 normalRotRight = manifold.normal.Rotated(-15 * DEG2RAD)*0.9f;
					glLoadIdentity();
					//Draw arrow of normal
					glTranslatef(currentPoint.x, currentPoint.y, -1);
					glBegin(GL_LINE_STRIP);
					glVertex3f(0, 0, 0);
					glVertex3f(manifold.normal.x, manifold.normal.y, 0);
					glVertex3f(normalRotLeft.x, normalRotLeft.y, 0);
					glVertex3f(normalRotRight.x, normalRotRight.y, 0);
					glVertex3f(manifold.normal.x, manifold.normal.y, 0);
					glEnd();
				}
			}
		}
		//Render grid
		if (m_drawGrid) {
			glColor3f(0, 255, 0);
			//Calculate the space we can see? To see how far to draw lines
			//Draw in positive and negative x, and positive and negative y
			glLoadIdentity();
			glTranslatef( 0, 0, -1);//Slightly in front of geometry
			glBegin(GL_LINE);
			glVertex3f(-100.f, 0, 0);
			glVertex3f(100.f, 0, 0);
			glVertex3f(0, -100.f, 0);
			glVertex3f(0, 100.f, 0);
			glColor3f(0, 200.f, 0);
			for (float i = 1; i < 100; i++) {
				//Horizontal
				glVertex3f(-100.f, i, 0);
				glVertex3f(100.f, i, 0);
				glVertex3f(-100.f, -i, 0);
				glVertex3f(100.f, -i, 0);
				//Vertical
				glVertex3f(i, -100.f, 0);
				glVertex3f(i, 100.f, 0);
				glVertex3f(-i, -100.f, 0);
				glVertex3f(-i, 100.f, 0);
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
		ImGui::Checkbox("Show Rigidbody Editor", &m_showRigidbodyEditor); 
		ImGui::Checkbox("Display manifolds", &m_displayManifolds);
		ImGui::Checkbox("Show Grid", &m_drawGrid);
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
	ImGui::Text("Rigidbodies");
	ImGui::Columns(2);
	ImGui::Separator();

	for (int i = 0; i < m_solver.m_rigidbodies.size(); i++) {
		Rigidbody * rb = m_solver.m_rigidbodies[i];
		std::string objShape;
		if (Circle* circle = dynamic_cast<Circle*>(rb)) {
			objShape = "Circle";
		}
		else if (Capsule* capsule = dynamic_cast<Capsule*>(rb)) {
			objShape = "Capsule";
		}
		else if (OrientedBox* orientedBox = dynamic_cast<OrientedBox*>(rb)) {
			objShape = "OrientedBox";
		}
		//Turn to char*
		char* strId = new char[10];
		snprintf(strId, 10, "Rb%i", i);//Worth revising this
		bool nodeOpen = ImGui::TreeNode(strId, "%s_%u", objShape.c_str(), i);
		ImGui::NextColumn();

		//Turn to char*
		char* objDesc = new char[100];
		snprintf(objDesc, 100, "X(%f), Y(%f)", rb->m_position.x, rb->m_position.y);//Worth revising this

		ImGui::Text(objDesc);
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

	ImGui::Text("Manifolds, contact point information");
	ImGui::Columns(2);
	ImGui::Separator();

	for (int i = 0; i < m_solver.m_currentManifolds.size(); i++) {
		//Turn to char*
		Manifold curManifold = m_solver.m_currentManifolds[i];
		char* strId = new char[10];
		snprintf(strId, 10, "Mf%i", i);//Worth revising this
		bool nodeOpen = ImGui::TreeNode(strId, "%s_%u", "Manifold", i);
		ImGui::NextColumn();

		std::string obj1;
		std::string obj2;
		//Turn to char*
		if (Circle* circle = dynamic_cast<Circle*>(curManifold.rb1)) {
			obj1 = "Circle";
		}
		else if (Capsule* capsule = dynamic_cast<Capsule*>(curManifold.rb1)) {
			obj1 = "Capsule";
		}
		else if (OrientedBox* orientedBox = dynamic_cast<OrientedBox*>(curManifold.rb1)) {
			obj1 = "OrientedBox";
		}

		if (Circle* circle = dynamic_cast<Circle*>(curManifold.rb2)) {
			obj2 = "Circle";
		}
		else if (Capsule* capsule = dynamic_cast<Capsule*>(curManifold.rb2)) {
			obj2 = "Capsule";
		}
		else if (OrientedBox* orientedBox = dynamic_cast<OrientedBox*>(curManifold.rb2)) {
			obj2 = "OrientedBox";
		}
		char* manifoldTitle = new char[100];
		snprintf(manifoldTitle, 100, "%s vs %s", obj1.c_str(), obj2.c_str());//Worth revising this

		ImGui::Text(manifoldTitle);
		ImGui::NextColumn();
		if (nodeOpen) {
			ImGui::Text("%s, position:", obj1.c_str());
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", curManifold.rb1->m_position.x, curManifold.rb1->m_position.y);
			ImGui::NextColumn();

			ImGui::Text("%s, position:", obj2.c_str());
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", curManifold.rb2->m_position.x, curManifold.rb2->m_position.y);
			ImGui::NextColumn();

			ImGui::Text("Penetration:");
			ImGui::NextColumn();
			ImGui::Text("%f", curManifold.penetration);
			ImGui::NextColumn();

			ImGui::Text("Normal:");
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", curManifold.normal.x, curManifold.normal.y);
			ImGui::NextColumn();

			for (int i = 0; i < curManifold.numContactPoints; i++) {
				ImGui::Text("ContactPoint_%i", i);
				ImGui::NextColumn();
				ImGui::Text("X(%f), Y(%f)", curManifold.contactPoints[i].x, curManifold.contactPoints[i].y);
				ImGui::NextColumn();
			}

			ImGui::TreePop();
		}
	}
	ImGui::Columns(1);
	ImGui::Separator();
	ImGui::End();
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
	if (m_inputPressed & KEY_F3)LoadScene(2);
	if (m_inputPressed & KEY_F4)LoadScene(3);
	if (m_inputPressed & KEY_R)m_solver.m_stepMode ^= 1;
	if (m_inputPressed & KEY_T)m_solver.m_stepOnce = m_solver.m_stepMode & true;
}
