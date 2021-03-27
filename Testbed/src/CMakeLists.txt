set (TESTBED_HEADER_FILES
	TestApp.h
	)
	
set(TESTBED_SOURCE_FILES
	main.cpp
	TestApp.cpp
	)
	
add_executable(Testbed ${TESTBED_HEADER_FILES} ${TESTBED_SOURCE_FILES})
target_link_libraries(Testbed fixedPoint)
target_link_libraries(Testbed pip)
target_link_libraries(Testbed imgui)
target_link_libraries(Testbed opengl32.lib)
target_link_libraries(Testbed glew32s.lib)
target_link_libraries(Testbed glfw3.lib)
target_link_libraries(Testbed ws2_32.lib)
target_link_libraries(Testbed userenv.lib)

target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../pip/src>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../pip/include>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../imgui/src>)
