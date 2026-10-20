# Interactive table tennis demo (Polyscope + ImPlot), gated behind PE_BUILD_TABLE_TENNIS_DEMO.
# Both dependencies are fetched at configure time (tools/cmake/ViewerDeps.cmake); nothing is
# added to the core build. Needs pe_CONSTRAINT_SOLVER =
# pe::response::HardContactSemiImplicitTimesteppingSolvers (restitution switch).

include(${CMAKE_SOURCE_DIR}/tools/cmake/ViewerDeps.cmake)

add_executable(pe_table_tennis
  table_tennis.cpp
)
# Polyscope's headers pull in glm/gtx/*; make the define explicit rather than
# relying on the polyscope target to propagate it.
target_compile_definitions(pe_table_tennis PRIVATE GLM_ENABLE_EXPERIMENTAL)
target_include_directories(pe_table_tennis PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/tools/contact_viewer   # ShapeMeshes.h (body-frame meshes of the primitives)
  ${MPI_CXX_INCLUDE_DIRS}
)
target_link_libraries(pe_table_tennis PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  polyscope
  implot
)
