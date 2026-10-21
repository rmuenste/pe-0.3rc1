# Impact lab (Polyscope + ImPlot), gated behind PE_BUILD_IMPACT_LAB: the two restitution schemes
# of the semi-implicit hard-contact solver side by side on multi-contact impacts. Same fetched
# dependencies as the other viewers (tools/cmake/ViewerDeps.cmake); nothing is added to the core
# build. Needs pe_CONSTRAINT_SOLVER = pe::response::HardContactSemiImplicitTimesteppingSolvers.

include(${CMAKE_SOURCE_DIR}/tools/cmake/ViewerDeps.cmake)

add_executable(pe_impact_lab
  impact_lab.cpp
)
target_compile_definitions(pe_impact_lab PRIVATE GLM_ENABLE_EXPERIMENTAL)
target_include_directories(pe_impact_lab PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/tools/contact_viewer   # ShapeMeshes.h (body-frame meshes of the primitives)
  ${MPI_CXX_INCLUDE_DIRS}
)
target_link_libraries(pe_impact_lab PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  polyscope
  implot
)
