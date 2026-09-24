# Interactive contact viewer (Polyscope + ImPlot), gated behind PE_BUILD_CONTACT_VIEWER.
# Both dependencies are fetched at configure time (tools/cmake/ViewerDeps.cmake); nothing is
# added to the core build.

include(${CMAKE_SOURCE_DIR}/tools/cmake/ViewerDeps.cmake)

add_executable(pe_contact_viewer
  contact_viewer.cpp
  pair_lab.cpp
  stack_lab.cpp
)
# Polyscope's headers pull in glm/gtx/*; make the define explicit rather than
# relying on the polyscope target to propagate it.
target_compile_definitions(pe_contact_viewer PRIVATE GLM_ENABLE_EXPERIMENTAL)
target_include_directories(pe_contact_viewer PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${MPI_CXX_INCLUDE_DIRS}
)
target_link_libraries(pe_contact_viewer PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  polyscope
  implot
)
