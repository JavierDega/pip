#define CATCH_CONFIG_RUNNER
#include "catch.h"
#include "TestApp.h"

using namespace math;

TEST_CASE("Base math queries") {
	REQUIRE( ClosestPtToSegment(Vector2(-1, 0), Vector2(1, 0), Vector2(0, 1)) == Vector2(0, 0) );
	REQUIRE( DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0) == Sqrt(2) );
}

int main(int argc, char* argv[])
{
	//Do tests here (asserts)
	int testResult = Catch::Session().run(argc, argv);
	TestApp testApp;
	if (testApp.Init() == -1) return -1;//Init error
	/* Loop until the user closes the window */
	testApp.UpdateLoop();
	return 0;
}