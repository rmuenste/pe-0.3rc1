add_executable(vtk_shapes_ascii vtk_shapes_ascii.cpp)
target_include_directories(vtk_shapes_ascii PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_compile_definitions(vtk_shapes_ascii PRIVATE VTK_SHAPES_ASCII_SRC_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
target_link_libraries(vtk_shapes_ascii PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
