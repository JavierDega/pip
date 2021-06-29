set(IMGUI_HEADER_FILES
	imconfig.h
	imgui.h
	imgui_impl_glfw.h
	imgui_impl_opengl3.h
	imgui_internal.h
	imstb_rectpack.h
	imstb_textedit.h
	imstb_truetype.h)
	
set(IMGUI_SOURCE_FILES
	imgui.cpp
	imgui_demo.cpp
	imgui_draw.cpp
	imgui_impl_glfw.cpp
	imgui_impl_opengl3.cpp
	imgui_widgets.cpp)

add_library(imgui ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES})