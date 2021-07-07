#define CATCH_CONFIG_RUNNER
#include "catch.h"

#include "TestApp.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

#define FP_EPSILON_TEMP 0.00001f
//Enable/Disable unit tests
#define RUN_TESTS 0

using namespace PipMath;
using namespace std;

//Unit Tests
TEST_CASE("Base math queries") {
	Vector2 segment1 = Vector2(-1, 0);
	Vector2 segment2 = Vector2(1, 0);
	Vector2 p = Vector2(0, 1);
	REQUIRE(ClosestPtToSegment(segment1, segment2, p).EqualsEps(Vector2(0.f, 0.f), FP_EPSILON_TEMP));
#if USE_FIXEDPOINT
	REQUIRE(DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0).EqualsEps( Sqrt(2), FP_EPSILON_TEMP));
#else
	REQUIRE(DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0) == Sqrt(2));
#endif
}

TEST_CASE("Collision response behavior") {
	//Mock objects
	Solver mockSolver;

	Circle circle1 = Circle(1.f, Vector2(-1, 0), 0.0f, Vector2(1, 0));
	Circle circle2 = Circle(1.f, Vector2(1, 0), 0.0f, Vector2(-1, 0));

	//Could run intersect test and check manifold data aswell rather than inputting manually
	Manifold testManifold;
	testManifold.contactPoints[0] = Vector2(0, 0);
	testManifold.numContactPoints = 1;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &circle1;
	testManifold.rb2 = &circle2;

	mockSolver.ComputeResponse(testManifold);

	REQUIRE((circle1.m_velocity == Vector2(-1, 0) && circle2.m_velocity == Vector2(1, 0)));
	
	OrientedBox obb1 = OrientedBox(Vector2(1.f, 1.f), Vector2(-1.f, 0.f), 0.f, Vector2(5.f, 1.f));
	OrientedBox obb2 = OrientedBox(Vector2(1.f, 1.f), Vector2(1.f, 0.f), 0.f, Vector2(-5.f, 1.f));

	testManifold.contactPoints[0] = Vector2(0, 1.0f);
	testManifold.contactPoints[1] = Vector2(0, -1.0f);
	testManifold.numContactPoints = 2;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &obb1;
	testManifold.rb2 = &obb2;

	mockSolver.ComputeResponse(testManifold);

	REQUIRE((obb1.m_velocity == Vector2(-5.f, 1.f) && obb2.m_velocity == Vector2(5.f, 1.f)));

	//#Possibly test collision detection logging aswell?
}

TEST_CASE("Colliders vs QuadNode intersect tests")
{
	//#Test non intersection?
	//Might be worth to test when QuadNode is much smaller than collider, even though its not designed to happen?
	//Sphere vs QuadNode
	Circle mockCircle = Circle(1.f, Vector2(0, 0));
	Vector2 topRight = Vector2(5, 5);
	Vector2 bottomLeft = Vector2(-5, -5);
	REQUIRE(mockCircle.IntersectWith(topRight, bottomLeft));
	mockCircle.m_position = Vector2(5 + Sqrt(0.5), 5 + Sqrt(0.5));
	mockCircle.m_position -= Vector2(FP_EPSILON_TEMP, FP_EPSILON_TEMP);
	REQUIRE(mockCircle.IntersectWith(topRight, bottomLeft));
	
	//All positions for capsule vs QuadNode
	Capsule mockCapsule = Capsule();
	mockCapsule.m_position = Vector2(-6, 0);
	REQUIRE(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_position = Vector2();
	mockCapsule.m_length = 12.1f;
	REQUIRE(mockCapsule.IntersectWith(topRight, bottomLeft));
	mockCapsule.m_rotation = 90 * DEG2RAD;
	REQUIRE(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_length = 15;
	mockCapsule.m_rotation = 45 * DEG2RAD;
	REQUIRE(mockCapsule.IntersectWith(topRight, bottomLeft));

	//Obb test (SAT)
	OrientedBox mockObb = OrientedBox();

	REQUIRE(mockObb.IntersectWith(topRight, bottomLeft));
	mockObb.m_position.x = -6.1f;
	REQUIRE(!mockObb.IntersectWith(topRight, bottomLeft));
}

int main(int argc, char* argv[])
{
	//Do tests here (asserts)
#if RUN_TESTS
	int testResult = Catch::Session().run(argc, argv);
#endif
	TestApp testApp;
	if (testApp.Init() == -1) {
		cout << "TestApp initialize failed" << endl;
		return -1;//Init error
	}
	/* Loop until the user closes the window */
	testApp.UpdateLoop();
	return 0;
}