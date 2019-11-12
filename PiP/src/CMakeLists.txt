set(PIP_SOURCE_FILES
	Rigidbody.cpp
	Solver.cpp
	StackAllocator.cpp)

set(PIP_HEADER_FILES
	PiPMath.h
	RigidBody.h
	Solver.h
	StackAllocator.h)

add_library(PiP ${PIP_SOURCE_FILES} ${PIP_HEADER_FILES})
target_link_libraries(PiP fixedPoint)