#define CATCH_CONFIG_RUNNER
#include "catch.h"

#include "TestApp.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"
#include "DefaultAllocator.h"
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
	//cout << "Test - ClosestPtToSegment: " << closestPt << endl;
	CHECK(EqualsEps(closestPt.x, 0, PIP_TEST_EPSILON));
	CHECK(EqualsEps(closestPt.y, 0, PIP_TEST_EPSILON));
	CHECK(EqualsEps(DistPtToPlane(Vector2(1, 1), Vector2(1, 1), 0), Sqrt(2), PIP_TEST_EPSILON));
}

TEST_CASE("Collision response behavior") {
	//Mock objects
	Solver mockSolver(new DefaultAllocator());

	Circle circle1 = Circle(1, Vector2(-1, 0), 0, Vector2(1, 0), 0.f, 1.f, 1.f);
	Circle circle2 = Circle(1, Vector2(1, 0), 0, Vector2(-1, 0), 0.f, 1.f, 1.f);

	//Could run intersect test and check manifold data aswell rather than inputting manually
	Manifold testManifold;
	testManifold.contactPoints[0] = Vector2(0, 0);
	testManifold.numContactPoints = 1;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &circle1;
	testManifold.rb2 = &circle2;

	mockSolver.ComputeResponse(testManifold);
	CHECK(circle1.m_velocity.EqualsEps(Vector2(-1, 0), PIP_TEST_EPSILON));
	CHECK(circle2.m_velocity.EqualsEps(Vector2(1, 0), PIP_TEST_EPSILON));
	CHECK((EqualsEps(circle1.m_velocity.x, -1, PIP_TEST_EPSILON) && EqualsEps(circle1.m_velocity.y, 0, PIP_TEST_EPSILON)));
	CHECK((EqualsEps(circle2.m_velocity.x, 1, PIP_TEST_EPSILON) && EqualsEps(circle2.m_velocity.y, 0, PIP_TEST_EPSILON)));


	OrientedBox obb1 = OrientedBox(Vector2(1, 1), Vector2(-1, 0), 0, Vector2(5, 1), 0.f, 1.f, 1.f);
	OrientedBox obb2 = OrientedBox(Vector2(1, 1), Vector2(1, 0), 0, Vector2(-5, 1), 0.f, 1.f, 1.f);

	testManifold.contactPoints[0] = Vector2(0, 1);
	testManifold.contactPoints[1] = Vector2(0, -1);
	testManifold.numContactPoints = 2;
	testManifold.normal = Vector2(-1, 0);
	testManifold.rb1 = &obb1;
	testManifold.rb2 = &obb2;

	mockSolver.ComputeResponse(testManifold);

	CHECK((EqualsEps(obb1.m_velocity.x, -5, PIP_TEST_EPSILON) && EqualsEps(obb1.m_velocity.y, 1, PIP_TEST_EPSILON)));
	CHECK((EqualsEps(obb2.m_velocity.x, 5, PIP_TEST_EPSILON) && EqualsEps(obb2.m_velocity.y, 1, PIP_TEST_EPSILON)));
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
	mockCircle.m_position = Vector2((decimal)5 + Sqrt(0.5f), (decimal)5 + Sqrt(0.5f));
	mockCircle.m_position -= Vector2(PIP_TEST_EPSILON, PIP_TEST_EPSILON);
	CHECK(mockCircle.IntersectWith(topRight, bottomLeft));
	
	//All positions for capsule vs QuadNode
	Capsule mockCapsule = Capsule();
	mockCapsule.m_position = Vector2(-6, 0);
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_position = Vector2();
	mockCapsule.m_length = 12.1f;
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));
	mockCapsule.m_rotation = 90 * PIP_DEG2RAD;
	CHECK(mockCapsule.IntersectWith(topRight, bottomLeft));

	mockCapsule.m_length = 15;
	mockCapsule.m_rotation = 45 * PIP_DEG2RAD;
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
	if (testResult != 0) 
	{
		cout << "Testbed Error - Main::Catch session run failed" << endl;
	}
#endif
	TestApp testApp;
	if (testApp.Init() == -1) {
		cout << "Testbed Error - Main::TestApp initialize failed" << endl;
		return -1;//Init error
	}
	/* Loop until the user closes the window */
	testApp.UpdateLoop();
	return 0;
}
