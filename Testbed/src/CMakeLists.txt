set (TESTBED_HEADER_FILES
	TestApp.h
	)
	
set(TESTBED_SOURCE_FILES
	Main.cpp
	TestApp.cpp
	)
	
add_executable(Testbed ${TESTBED_HEADER_FILES} ${TESTBED_SOURCE_FILES})
target_link_libraries(Testbed fixedPoint)
target_link_libraries(Testbed PiP)
target_link_libraries(Testbed Imgui)
target_link_libraries(Testbed opengl32.lib)
target_link_libraries(Testbed glew32s.lib)
target_link_libraries(Testbed glfw3.lib)
target_link_libraries(Testbed ws2_32.lib)
target_link_libraries(Testbed userenv.lib)

target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../PiP/src>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../PiP/include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../Imgui/src>)
