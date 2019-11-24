
#include "TestApp.h"

int main(void)
{
	//Do tests here (asserts)
	TestApp testApp;
	if (testApp.Init() == -1) return -1;//Init error
	/* Loop until the user closes the window */
	testApp.UpdateLoop();
	return 0;
}