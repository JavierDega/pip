#define CATCH_CONFIG_RUNNER
#include "catch.h"

#include "TestApp.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

//Enable/Disable unit tests
#define RUN_TESTS 1

using namespace PipMath;
using namespace std;

//Unit Tests
TEST_CASE("Base math queries") {
	Vector2 a = Vector2(-1, 0);
	Vector2 b = Vector2(1, 0);
	Vector2 p = Vector2(0, 1);
	Vector2 closestPt = ClosestPtToSegment(a, b, p);
	cout << closestPt << endl;
	CHECK((float)closestPt.x == Approx(0));
	CHECK((float)closestPt.y == Approx(0));
	CHECK((float)DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0) == Approx(Sqrt(2)));
}

TEST_CASE("Collision response behavior") {
	//Mock objects
	Solver mockSolver;

	Circle circle1 = Circle(1, Vector2(-1, 0), 0, Vector2(1, 0));
	Circle circle2 = Circle(1, Vector2(1, 0), 0, Vector2(-1, 0));

	//Could run intersect test and check manifold data aswell rather than inputting manually
	Manifold testManifold;
	testManifold.contactPoints[0] = Vector2(0, 0);
	testManifold.numContactPoints = 1;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &circle1;
	testManifold.rb2 = &circle2;

	mockSolver.ComputeResponse(testManifold);
	CHECK((circle1.m_velocity.x == Approx(-1) && circle1.m_velocity.y == Approx(0)));
	CHECK((circle2.m_velocity.x == Approx(1) && circle2.m_velocity.y == Approx(0)));
	
	OrientedBox obb1 = OrientedBox(Vector2(1, 1), Vector2(-1, 0), 0, Vector2(5, 1));
	OrientedBox obb2 = OrientedBox(Vector2(1, 1), Vector2(1, 0), 0, Vector2(-5, 1));

	testManifold.contactPoints[0] = Vector2(0, 1);
	testManifold.contactPoints[1] = Vector2(0, -1);
	testManifold.numContactPoints = 2;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &obb1;
	testManifold.rb2 = &obb2;

	mockSolver.ComputeResponse(testManifold);

	CHECK((obb1.m_velocity.x == Approx(-5) && obb1.m_velocity.y == Approx(1)));
	CHECK((obb2.m_velocity.x == Approx(5) && obb2.m_velocity.y == Approx(1)));
	//#Possibly test collision detection logging aswell?
}

TEST_CASE("Colliders vs QuadNode intersect tests")
{
	//#Test non intersection?
	//Might be worth to test when QuadNode is much smaller than collider, even though its not designed to happen?
	//Sphere vs QuadNode
	Circle mockCircle = Circle(1, Vector2(0, 0));
	Vector2 topRight = Vector2(5, 5);
	Vector2 bottomLeft = Vector2(-5, -5);
	CHECK(mockCircle.IntersectWith(topRight, bottomLeft));
	mockCircle.m_position = Vector2(5 + Sqrt(0.5f), 5 + Sqrt(0.5f));
	mockCircle.m_position -= Vector2(FLT_EPSILON_TESTS, FLT_EPSILON_TESTS);
	CHECK(mockCircle.IntersectWith(topRight, bottomLeft));
	
	//All positions for capsule vs QuadNode
	Capsule mockCapsule = Capsule();
	mockCapsule.m_position = Vector2(-6, 0);
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_position = Vector2();
	mockCapsule.m_length = 12.1f;
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));
	mockCapsule.m_rotation = 90 * DEG2RAD;
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_length = 15;
	mockCapsule.m_rotation = 45 * DEG2RAD;
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));

	//Obb test (SAT)
	OrientedBox mockObb = OrientedBox();

	CHECK(mockObb.IntersectWith(topRight, bottomLeft));
	mockObb.m_position.x = -6.1f;
	CHECK(!mockObb.IntersectWith(topRight, bottomLeft));
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