# Shared third-party dependencies of the interactive Polyscope tools (tools/live_viewer,
# tools/contact_viewer). Fetched at configure time; nothing is added to the core build.
# Included by each tool's CMakeLists.txt so all viewers use the same pinned versions.

include_guard(GLOBAL)

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
