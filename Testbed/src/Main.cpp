#define CATCH_CONFIG_RUNNER
#include "catch.h"
#include "TestApp.h"
#include "Circle.h"
#include "OrientedBox.h"
using namespace math;

TEST_CASE("Base math queries") {
	REQUIRE( ClosestPtToSegment(Vector2(-1, 0), Vector2(1, 0), Vector2(0, 1)) == Vector2(0, 0) );
	REQUIRE( DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0) == Sqrt(2) );
}

TEST_CASE("Collision response behavior") {
	//Mock objects
	Solver* mockSolver = new Solver();

	Circle* circle1 = new Circle(Vector2(-1, 0), 0.0f, Vector2(1, 0));
	Circle* circle2 = new Circle(Vector2(1, 0), 0.0f, Vector2(-1, 0));

	Manifold testManifold;
	testManifold.contactPoints[0] = Vector2(0, 0);
	testManifold.numContactPoints = 1;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = circle1;
	testManifold.rb2 = circle2;

	mockSolver->ComputeResponse(testManifold);

	REQUIRE( (circle1->m_velocity == Vector2(-1, 0) && circle2->m_velocity == Vector2(1, 0)) );

	delete circle1; circle1 = nullptr;
	delete circle2, circle2 = nullptr;
	
	OrientedBox* obb1 = new OrientedBox(Vector2(-1.f, 0.f), 0.f, Vector2(5.f, 1.f));
	OrientedBox* obb2 = new OrientedBox(Vector2(1.f, 0.f), 0.f, Vector2(-5.f, 1.f));

	testManifold.contactPoints[0] = Vector2(0, 1.0f);
	testManifold.contactPoints[1] = Vector2(0, -1.0f);
	testManifold.numContactPoints = 2;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = obb1;
	testManifold.rb2 = obb2;

	mockSolver->ComputeResponse(testManifold);

	REQUIRE((obb1->m_velocity == Vector2(-5.f, 1.f) && obb2->m_velocity == Vector2(5.f, 1.f)));

	delete obb1; obb1 = nullptr;
	delete obb2; obb2 = nullptr;

	delete mockSolver; mockSolver = nullptr;
	//Possibly test collision detection logging aswell?
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