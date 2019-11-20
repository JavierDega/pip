set(PIP_SOURCE_FILES
	Rigidbody.cpp
	Circle.cpp
	Capsule.cpp
	Solver.cpp
	StackAllocator.cpp)

set(PIP_HEADER_FILES
	PiPMath.h
	RigidBody.h
	Circle.h
	Capsule.h
	Solver.h
	StackAllocator.h)

add_library(PiP ${PIP_HEADER_FILES} ${PIP_SOURCE_FILES})