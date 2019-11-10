add_subdirectory(src)

target_include_directories(Testbed 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(Testbed 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../PiP/src>)
target_include_directories(Testbed 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../PiP/include>)

target_link_directories(Testbed 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>)
