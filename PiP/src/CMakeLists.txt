set(PIP_HEADER_FILES
	PiPMath.h
	Rigidbody.h
	Circle.h
	Capsule.h
	OrientedBox.h
	Solver.h
	DefaultAllocator.h
	QuadNode.h)
	
set(PIP_SOURCE_FILES
	Rigidbody.cpp
	Circle.cpp
	Capsule.cpp
	OrientedBox.cpp
	Solver.cpp
	DefaultAllocator.cpp
	QuadNode.cpp)

add_library(PiP ${PIP_HEADER_FILES} ${PIP_SOURCE_FILES})