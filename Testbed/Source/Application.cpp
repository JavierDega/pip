#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "Solver.h"

int main(void)
{
	GLFWwindow* window;

	/* Initialize the library */
	if (!glfwInit())
		return -1;
	
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	if (glewInit() != GLEW_OK)
		std::cout << "Error!" << std::endl;
	std::cout << glGetString(GL_VERSION) << std::endl;

	//Create solver to loop through
	Solver * solver = new Solver();
	solver->AddBody();
	decimal prevTime = glfwGetTime();
	/* Loop until the user closes the window */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//				Identity	fov		aspect ratio nearVal farVal
	//guPerspective(m_projection, 45, (f32)w / h, 0.1F, 1000.0F);
	/*fov = DEG_TO_RAD(fov);                      // transform fov from degrees to radians
	float tangent = tanf(fov / 2.0f);               // tangent of half vertical fov
	float height = front * tangent;                 // half height of near plane
	float width = height * aspect;                  // half width of near plane
	*/
	glFrustum(-1, 1, -1, 1, -1, 500);
	glMatrixMode(GL_MODELVIEW);
	glFrontFace(GL_CCW);//Specify backface culling (Clockwise/ counter clockwise);
	while (!glfwWindowShouldClose(window))
	{
		decimal curTime = glfwGetTime();
		decimal dt = curTime - prevTime;
		prevTime = curTime;

		/*Physics update*/
		solver->Update(dt);
		/* Render here */
		glClear(GL_COLOR_BUFFER_BIT);
		for (Rigidbody* rb : solver->m_rigidbodies) {
			//Draw
			glLoadIdentity();
			glScalef((float)rb->m_radius, (float)rb->m_radius, (float)rb->m_radius);
			glRotatef((float)rb->m_rotation, 0, 0, 1);
			glTranslatef((float)rb->m_position.x, (float)rb->m_position.y, 0);
			glBegin(GL_TRIANGLES);
			
			//Circle vertices from trig
			for (int i = 0; i < 350; i += 10) {
				glVertex3f(0, 0, 0);
				glVertex3f(cos(i * PI / 180.f), sin(i * PI / 180.f), 0);
				glVertex3f(cos((i + 10.f) * PI / 180.f), sin((i + 10.f) * PI / 180.f), 0);
			}

			glEnd();
		}
		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}