include_directories(include)
link_directories(lib)
add_library(fixedPoint STATIC IMPORTED)
set_target_properties(fixedPoint PROPERTIES IMPORTED_LOCATION
 ${CMAKE_CURRENT_SOURCE_DIR}/lib/fp_math_bindings.lib)
add_subdirectory(src)
