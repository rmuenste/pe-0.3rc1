# pe ports of LIGGGHTS-PUBLIC tutorials (see LIGGGHTS-PUBLIC/md_docs/RUNBOOK.md)
function(lp_add_example target src)
  add_executable(${target} ${src})
  target_include_directories(${target} PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${target} PRIVATE
    $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
    $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
    $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  )
endfunction()

lp_add_example(lp_insert_stream insert_stream.cpp)
lp_add_example(lp_cylinder_pack cylinder_pack.cpp)
lp_add_example(lp_multisphere   multisphere.cpp)

if(PE_USE_CGAL)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mesh_gran.cpp)
    lp_add_example(lp_mesh_gran mesh_gran.cpp)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/moving_mesh_gran.cpp)
    lp_add_example(lp_moving_mesh_gran moving_mesh_gran.cpp)
  endif()
endif()
