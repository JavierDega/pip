set(TESTBED_SOURCE_FILES
	Application.cpp)

add_executable(Testbed ${TESTBED_SOURCE_FILES})
target_link_libraries(Testbed PiP)
target_link_libraries(Testbed glew32s.lib)
target_link_libraries(Testbed glfw3.lib)

target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../PiP/src>)
target_include_directories(Testbed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../PiP/include>)