add_subdirectory(src)

target_include_directories(PiP 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_directories(PiP 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>)
