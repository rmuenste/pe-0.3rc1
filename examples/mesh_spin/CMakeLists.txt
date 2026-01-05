# Mesh spin example - requires CGAL for DistanceMap support

if(CGAL)
  add_executable(mesh_spin mesh_spin.cpp)

  target_include_directories(mesh_spin PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
  target_link_libraries(mesh_spin PRIVATE 
    $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
    $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
    $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
  )


  message(STATUS "Configured mesh_spin example (requires CGAL)")
else()
  message(STATUS "Skipping mesh_spin example (requires CGAL)")
endif()
