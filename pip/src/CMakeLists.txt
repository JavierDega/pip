set(PIP_HEADER_FILES
	PipMath.h
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

add_library(pip ${PIP_HEADER_FILES} ${PIP_SOURCE_FILES})