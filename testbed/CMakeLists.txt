include_directories(include)
link_directories(lib)
add_library(fixedPoint STATIC IMPORTED)
if (WIN32)
    set_target_properties(fixedPoint PROPERTIES IMPORTED_LOCATION
     ${CMAKE_BINARY_DIR}/../pip/lib/fp_math_bindings.lib)
endif (WIN32)
if (UNIX)
    set_target_properties(fixedPoint PROPERTIES IMPORTED_LOCATION
     ${CMAKE_BINARY_DIR}/../pip/lib/libfp_math_bindings.a)
endif (UNIX)
add_compile_definitions(GLEW_STATIC)
add_subdirectory(src)


