#include "TestApp.h"

#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

using namespace pipmath;

TestApp::TestApp()
	:m_window(nullptr), m_glslVersion(""), m_sceneName(""), m_prevTime(0), m_showDemoWindow(false), m_showRigidbodyEditor(true), m_displayManifolds(true), m_drawGrid(true),
	m_renderLeafNodes(true), m_inputDown(0), m_inputPressed(0), m_inputHeld(0), m_inputReleased(0)
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
	m_window = glfwCreateWindow(800, 800, "PiP physics demo", NULL, NULL);
	if (!m_window){
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(m_window);
	//glfwSwapInterval(1); // Enable vsync
	if (glewInit() != GLEW_OK)
		std::cout << "Error! glewInit() != GLEW_OK" << std::endl;
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
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(m_window, true);
	ImGui_ImplOpenGL3_Init(m_glslVersion);
}

void TestApp::LoadScene(unsigned int index)
{
	//Deallocate m_allocator pool
	m_solver.m_allocator.DestroyAllBodies();
	m_solver.m_currentManifolds.clear();
	m_bodyHandles.clear();
	switch (index) {
	case 0:
	{
		m_sceneName = "Scene 0: Circles v Capsule";
		m_bodyHandles.push_back(m_solver.CreateCircle(1.0f, Vector2(-5, 6), 0, Vector2(5, 0)));
		m_bodyHandles.push_back(m_solver.CreateCapsule(2.f, 1.0f, Vector2(), 0 * DEG2RAD, Vector2(), 0.f, 1.f, 0.9f, true));
		m_bodyHandles.push_back(m_solver.CreateCircle(1.0f, Vector2(5, 5), 0, Vector2(-5, 0)));
		break;
	}
	case 1:
	{
		m_sceneName = "Scene 1: Capsule v OrientedBox";
		m_bodyHandles.push_back(m_solver.CreateCapsule(2.f, 1.f, Vector2(0, 5), 45 * DEG2RAD));
		m_bodyHandles.push_back(m_solver.CreateOrientedBox(Vector2(2, 2), Vector2(0, -2), 45 * DEG2RAD, Vector2(0, 0), 0.0f, 100.f));
		break;
	}
	case 2:
	{
		m_sceneName = "Scene 2: Sphere against Obb";
		m_bodyHandles.push_back(m_solver.CreateCircle(1.0f, Vector2(0.1f, 5)));
		m_bodyHandles.push_back(m_solver.CreateOrientedBox(Vector2(1.f, 1.f), Vector2(3, 0), 0 * DEG2RAD, Vector2(), 0.0f, 100.f));
		m_bodyHandles.push_back(m_solver.CreateOrientedBox(Vector2(0.5f, 0.5f), Vector2(0, 0), 45 * DEG2RAD, Vector2(), 0.0f, 100.f));
		break;
	}
	case 3:
	{
		m_sceneName = "Scene 3: OBB collision with SAT, uses discontiguous std::vector";
		m_bodyHandles.push_back(m_solver.CreateOrientedBox(Vector2(1.f, 1.f), Vector2(-5, 5), 0 * DEG2RAD, Vector2(5, 0), 0.0f, 100.f));
		m_bodyHandles.push_back(m_solver.CreateOrientedBox(Vector2(1.f, 1.f), Vector2(5, 5), 0 * DEG2RAD, Vector2(-5, 0), 0.0f, 100.f));
		break;
	}
	case 4:
	{
		m_sceneName = "Scene 4: Capsule to capsule";
		m_bodyHandles.push_back(m_solver.CreateCapsule(1.f, 1.f, Vector2(0, 4), 0 * DEG2RAD, Vector2(), 0.f, 1.f, 0.9f));
		m_bodyHandles.push_back(m_solver.CreateCapsule(4.f, 1.f, Vector2(0, -2), 0 * DEG2RAD, Vector2(), 0.0f, 1.f, 0.7f, true));
		break;
	}
	case 5:
	{
		m_sceneName = "Scene 5: Testing QuadTree";
		m_bodyHandles.push_back(m_solver.CreateCapsule(16.f, 1.f, Vector2(0, -9), 0 * DEG2RAD, Vector2(), 0.0f, 1.f, 0.7f, true));
		//Circles
		m_bodyHandles.push_back(m_solver.CreateCircle(1.0f, Vector2(-2.f, -4), 0.0f, Vector2(0.5f, 0)));
		break;
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
		glColor3f(1, 1, 1);
		//Loop through solver's rigidbody pool
		for (Rigidbody* rb = (Rigidbody*)m_solver.m_allocator.GetFirstBody(); rb != nullptr; rb = m_solver.m_allocator.GetNextBody(rb)) {
			glLoadIdentity();
			switch (rb->m_bodyType) {
			case BodyType::Circle: {
				Circle* circle = (Circle*)rb;
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
				break;
			}
			case BodyType::Capsule: {
				Capsule* capsule = (Capsule*)rb;
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
				break;
			}
			case BodyType::Obb: {
				OrientedBox* obb = (OrientedBox*)rb;
				glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, -1);
				glRotatef((float)rb->m_rotation * RAD2DEG, 0, 0, 1);
				glBegin(GL_TRIANGLES);
				//Rectangle made up of two triangles
				Vector2 halfExtents = obb->m_halfExtents;
				glVertex3f(-(float)halfExtents.x, -(float)halfExtents.y, 0);
				glVertex3f((float)halfExtents.x, -(float)halfExtents.y, 0);
				glVertex3f((float)halfExtents.x, (float)halfExtents.y, 0);
				//Upper tri
				glVertex3f(-(float)halfExtents.x, -(float)halfExtents.y, 0);
				glVertex3f((float)halfExtents.x, (float)halfExtents.y, 0);
				glVertex3f(-(float)halfExtents.x, (float)halfExtents.y, 0);
				break;
			}
			}
			glEnd();
		}

		//Render manifolds
		if (m_displayManifolds) {
			glColor3f(1, 0, 0);
			//Draw in red: Normal with magnitude at all contact points
			for ( Manifold& manifold : m_solver.m_currentManifolds) {
				for (int i = 0; i < manifold.numContactPoints; i++) {
					Vector2 currentPoint = manifold.contactPoints[i];
					Vector2 normalRotLeft = manifold.normal.Rotated(15 * DEG2RAD)*0.9f;
					Vector2 normalRotRight = manifold.normal.Rotated(-15 * DEG2RAD)*0.9f;
					glLoadIdentity();
					//Draw arrow of normal
					glTranslatef((float)currentPoint.x, (float)currentPoint.y, -1);
					glBegin(GL_LINE_STRIP);
					glVertex3f(0, 0, 0);
					glVertex3f((float)manifold.normal.x, (float)manifold.normal.y, 0);
					glVertex3f((float)normalRotLeft.x, (float)normalRotLeft.y, 0);
					glVertex3f((float)normalRotRight.x, (float)normalRotRight.y, 0);
					glVertex3f((float)manifold.normal.x, (float)manifold.normal.y, 0);
					glEnd();
				}
			}
		}
		//Render grid
		if (m_drawGrid) {
			glColor3f(0, 1, 0);
			//Calculate the space we can see? To see how far to draw lines
			//Draw in positive and negative x, and positive and negative y
			glLoadIdentity();
			glTranslatef( 0, 0, -1);//Slightly in front of geometry
			glBegin(GL_LINES);
			glVertex3f(-100.f, 0, 0);
			glVertex3f(100.f, 0, 0);
			glVertex3f(0, -100.f, 0);
			glVertex3f(0, 100.f, 0);
			glEnd();
			glBegin(GL_LINES);
			glColor3f(0, 0.5f, 0);
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
		//Render leaf nodes
		if (m_renderLeafNodes)
		{
			//Similar to drawing grid
			std::vector<QuadNode*> leafNodes;
			m_solver.m_quadTreeRoot.GetLeafNodes(leafNodes);
			glColor3f(1, 0, 0);
			glLoadIdentity();
			glTranslatef(0, 0, -1);//Slightly in front of geometry
			glBegin(GL_LINES);
			for (int i = 0; i < leafNodes.size(); i++)
			{
				QuadNode* currentLeaf = leafNodes[i];
				Vector2 topRight = currentLeaf->m_topRight;
				Vector2 bottomLeft = currentLeaf->m_bottomLeft;
				glVertex3f((float)topRight.x, (float)topRight.y, 0);
				glVertex3f((float)bottomLeft.x, (float)topRight.y, 0);

				glVertex3f((float)bottomLeft.x, (float)topRight.y, 0);
				glVertex3f((float)bottomLeft.x, (float)bottomLeft.y, 0);

				glVertex3f((float)bottomLeft.x, (float)bottomLeft.y, 0);
				glVertex3f((float)topRight.x, (float)bottomLeft.y, 0);

				glVertex3f((float)topRight.x, (float)bottomLeft.y, 0);
				glVertex3f((float)topRight.x, (float)topRight.y, 0);

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
		ImGui::Text("Press F1-F10 to load scenes");
		ImGui::Checkbox("Step mode (R)", &m_solver.m_stepMode);
		ImGui::Checkbox("Step once (T)", &m_solver.m_stepOnce);
		ImGui::Text("Continuous Collision : False");
		ImGui::Checkbox("Demo Window", &m_showDemoWindow);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Log Collision Info", &m_solver.m_logCollisionInfo);
		ImGui::Checkbox("Static & Kinetic friction", &m_solver.m_frictionModel);
		ImGui::Checkbox("Show Rigidbody Editor (Y)", &m_showRigidbodyEditor); 
		ImGui::Checkbox("Display manifolds (U)", &m_displayManifolds);
		ImGui::Checkbox("Show Grid (I)", &m_drawGrid);
		ImGui::Text("Destroy first body (O)");
		ImGui::Text("Create circle (P)");
		ImGui::Text("Ignore separating bodies: True");
		ImGui::Text("Static collision resolution: True");
		ImGui::Checkbox("Show Leaf Nodes", &m_renderLeafNodes);
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
	int i = 0;
	for (Rigidbody* rb = (Rigidbody*)m_solver.m_allocator.GetFirstBody(); rb != nullptr; rb = m_solver.m_allocator.GetNextBody(rb)) {
		std::string objShape;
		char* objDesc = new char[100];
		switch (rb->m_bodyType) {
		case BodyType::Circle: {
			Circle* circle = (Circle*)rb;
			objShape = "Circle";
			snprintf(objDesc, 100, "Radius(%f)", (double)circle->m_radius);
			break;
		}
		case BodyType::Capsule: {
			Capsule* capsule = (Capsule*)rb;
			objShape = "Capsule";
			snprintf(objDesc, 100, "Radius(%f), Length(%f)", (double)capsule->m_radius, (double)capsule->m_length);
			break;
		}
		case BodyType::Obb: {
			OrientedBox* obb = (OrientedBox*)rb;
			objShape = "OrientedBox";
			snprintf(objDesc, 100, "halfExtents: x(%f), y(%f)", (double)obb->m_halfExtents.x, (double)obb->m_halfExtents.y);//Worth revising this
			break;
		}
		}
		//Turn to char*
		char* strId = new char[10];
		snprintf(strId, 10, "Rb%i", i);//Worth revising this
		bool nodeOpen = ImGui::TreeNode(strId, "%s_%u Pos: x(%f), y(%f)", objShape.c_str(), i, (double)rb->m_position.x, (double)rb->m_position.y);
		ImGui::NextColumn();
		ImGui::Text("%s", objDesc);
		ImGui::NextColumn();
		if (nodeOpen)
		{
			//Show rotation, velocity, angular velocity
			ImGui::Text("Rotation");
			ImGui::NextColumn();
			char rotation[50];
			snprintf(rotation, 50, "Rad(%f), Deg(%f)", (double)rb->m_rotation, (double)rb->m_rotation * RAD2DEG);
			ImGui::Text("%s", rotation);
			ImGui::NextColumn();

			ImGui::Text("Velocity");
			ImGui::NextColumn();
#if USE_FIXEDPOINT
			char realVel[50];
			snprintf(realVel, 50, "Vel (Real) X(%f), Y(%f)", (double)rb->m_velocity.x, (double)rb->m_velocity.y);
			ImGui::Text(realVel);
#else
			ImGui::DragFloat("VelX", &rb->m_velocity.x, 1.0f);//#TODO: Might not be compatible with fixedpoint mode. Create wrapper for inputfloat funcs?
			ImGui::DragFloat("VelY", &rb->m_velocity.y, 1.0f);
#endif
			ImGui::NextColumn();

			ImGui::Text("AngVel");
			ImGui::NextColumn();
#if USE_FIXEDPOINT
			char realRot[50];
			snprintf(realRot, 50, "Rot (Real) (%f)", (double)rb->m_angularVelocity);
			ImGui::Text(realRot);
#else
			ImGui::DragFloat("Rot (Rad/S)", &rb->m_angularVelocity, 0.1f);
#endif
			ImGui::NextColumn();

			ImGui::Text("Acceleration");
			ImGui::NextColumn();
#if USE_FIXEDPOINT
			char realAccel[50];
			snprintf(realAccel, 50, "Accel (Real) X(%f) Y(%f)", (double)rb->m_acceleration.x, (double)rb->m_acceleration.y);
			ImGui::Text(realAccel);
#else

			ImGui::DragFloat("AccelX", &rb->m_acceleration.x, 1.0f);
			ImGui::DragFloat("AccelY", &rb->m_acceleration.y, 1.0f);
#endif
			ImGui::NextColumn();
			ImGui::Text("Mass");
			ImGui::NextColumn();
			char mass[50];
			snprintf(mass, 50, "%f", (float)rb->m_mass);
			ImGui::Text("%s", mass);
			ImGui::NextColumn();


			ImGui::Text("Inertia");
			ImGui::NextColumn();
			char inertia[50];
			snprintf(inertia, 50, "%f", (float)rb->m_inertia);
			ImGui::Text("%s", inertia);
			ImGui::NextColumn();

			ImGui::Text("Kinematic");
			ImGui::NextColumn();
			ImGui::Checkbox("Is Kinematic?", &rb->m_isKinematic);
			ImGui::NextColumn();

			ImGui::Text("Sleeping");
			ImGui::NextColumn();
			ImGui::Checkbox("Is Sleeping?", &rb->m_isSleeping);
			ImGui::NextColumn();
			ImGui::TreePop();
		}
		i++;
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

		ImGui::Text("%s", manifoldTitle);
		ImGui::NextColumn();
		if (nodeOpen) {
			ImGui::Text("%s, position:", obj1.c_str());
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", (double)curManifold.rb1->m_position.x, (double)curManifold.rb1->m_position.y);
			ImGui::NextColumn();

			ImGui::Text("%s, position:", obj2.c_str());
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", (double)curManifold.rb2->m_position.x, (double)curManifold.rb2->m_position.y);
			ImGui::NextColumn();

			ImGui::Text("Penetration:");
			ImGui::NextColumn();
			ImGui::Text("%f", (double)curManifold.penetration);
			ImGui::NextColumn();

			ImGui::Text("Normal:");
			ImGui::NextColumn();
			ImGui::Text("X(%f), Y(%f)", (double)curManifold.normal.x, (double)curManifold.normal.y);
			ImGui::NextColumn();

			for (int i = 0; i < curManifold.numContactPoints; i++) {
				ImGui::Text("ContactPoint_%i", i);
				ImGui::NextColumn();
				ImGui::Text("X(%f), Y(%f)", (double)curManifold.contactPoints[i].x, (double)curManifold.contactPoints[i].y);
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
	short f6 = glfwGetKey(m_window, GLFW_KEY_F6);
	short r = glfwGetKey(m_window, GLFW_KEY_R);
	short t = glfwGetKey(m_window, GLFW_KEY_T);
	short y = glfwGetKey(m_window, GLFW_KEY_Y);
	short u = glfwGetKey(m_window, GLFW_KEY_U);
	short i = glfwGetKey(m_window, GLFW_KEY_I);
	short o = glfwGetKey(m_window, GLFW_KEY_O);
	short p = glfwGetKey(m_window, GLFW_KEY_P);

	short inputDownNew = (f1 << 0) | (f2 << 1) | (f3 << 2) | (f4 << 3) | (f5 << 4) | (f6 << 5) | (r << 6) | (t << 7) | (y << 8) | (u << 9) | (i << 10) | (o << 11) | (p << 12);
	//AND with m_inputDown to get m_inputHeld
	m_inputHeld = m_inputDown & inputDownNew;
	m_inputPressed = ~m_inputDown & inputDownNew;
	//etc, then assign new inputDown
	m_inputReleased = m_inputDown & ~inputDownNew;
	m_inputDown = inputDownNew;

	//React to input
	if (m_inputPressed & (short)Keys::F1)LoadScene(0);
	if (m_inputPressed & (short)Keys::F2)LoadScene(1);
	if (m_inputPressed & (short)Keys::F3)LoadScene(2);
	if (m_inputPressed & (short)Keys::F4)LoadScene(3);
	if (m_inputPressed & (short)Keys::F5)LoadScene(4);
	if (m_inputPressed & (short)Keys::F6)LoadScene(5);

	if (m_inputPressed & (short)Keys::R)m_solver.m_stepMode ^= 1;
	if (m_inputPressed & (short)Keys::T)m_solver.m_stepOnce = m_solver.m_stepMode & true;
	if (m_inputPressed & (short)Keys::Y)m_showRigidbodyEditor ^= 1;
	if (m_inputPressed & (short)Keys::U)m_displayManifolds ^= 1;
	if (m_inputPressed & (short)Keys::I)m_drawGrid ^= 1;
	if (m_inputPressed & (short)Keys::O) 
	{
		//Debug delete first body handle on the list
		if (!m_bodyHandles.empty()){
			m_solver.m_allocator.DestroyBody(m_bodyHandles[0]);
			m_bodyHandles.erase(m_bodyHandles.begin());
			//#WIP Solution to manifolds being invalid when deleting objs on step mode, as Step() doesn't run to clear them
			if (m_solver.m_stepMode) m_solver.m_currentManifolds.clear();
		}
	}
	if (m_inputPressed & (short)Keys::P) m_bodyHandles.push_back(m_solver.CreateCircle());//Debug add cirlce at (0, 0)
}
