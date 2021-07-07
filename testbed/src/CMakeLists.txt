set (TESTBED_HEADER_FILES
	TestApp.h
	)
	
set(TESTBED_SOURCE_FILES
	Main.cpp
	TestApp.cpp
	)
	
add_executable(Testbed ${TESTBED_HEADER_FILES} ${TESTBED_SOURCE_FILES})
target_link_libraries(Testbed fixedPoint)
target_link_libraries(Testbed pip)
target_link_libraries(Testbed imgui)
if (WIN32)
	target_link_libraries(Testbed opengl32.lib)
	target_link_libraries(Testbed glew32s.lib)
	target_link_libraries(Testbed glfw3.lib)
	target_link_libraries(Testbed ws2_32.lib)
	target_link_libraries(Testbed userenv.lib)
endif (WIN32)
if (UNIX)
	find_package(glfw3 REQUIRED)
	find_package(OpenGL REQUIRED)
	find_package(GLEW REQUIRED)
	target_link_libraries(Testbed glfw)
	target_link_libraries(Testbed GL)
	target_link_libraries(Testbed GLEW)
	include_directories(${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})
	link_libraries(${GLEW_LIBRARIES} ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES})
endif (UNIX)


target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../pip/src>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../pip/include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../imgui/src>)
