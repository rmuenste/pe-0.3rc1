# Interactive live viewer (Polyscope + ImPlot), gated behind PE_BUILD_LIVE_VIEWER.
# Both dependencies are fetched at configure time; nothing is added to the core build.

include(FetchContent)

FetchContent_Declare(polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG        v2.3.0
)
FetchContent_MakeAvailable(polyscope)

# ImPlot ships no CMakeLists; compile it against Polyscope's bundled imgui target
# so both use the same ImGui context.
FetchContent_Declare(implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        v0.16
)
FetchContent_MakeAvailable(implot)

add_library(implot STATIC
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

add_executable(pe_live_viewer
  live_viewer.cpp
)
# Polyscope's headers pull in glm/gtx/*; make the define explicit rather than
# relying on the polyscope target to propagate it.
target_compile_definitions(pe_live_viewer PRIVATE GLM_ENABLE_EXPERIMENTAL)
target_include_directories(pe_live_viewer PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${MPI_CXX_INCLUDE_DIRS}
)
target_link_libraries(pe_live_viewer PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  polyscope
  implot
)
