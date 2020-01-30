set(PIP_HEADER_FILES
	PiPMath.h
	RigidBody.h
	Circle.h
	Capsule.h
	OrientedBox.h
	Solver.h
	StackAllocator.h)
	
set(PIP_SOURCE_FILES
	Rigidbody.cpp
	Circle.cpp
	Capsule.cpp
	OrientedBox.cpp
	Solver.cpp
	StackAllocator.cpp)

add_library(PiP ${PIP_HEADER_FILES} ${PIP_SOURCE_FILES})