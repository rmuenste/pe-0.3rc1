add_executable(triangle_mesh triangle_mesh.cpp)
target_include_directories(triangle_mesh PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(triangle_mesh PRIVATE 
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)