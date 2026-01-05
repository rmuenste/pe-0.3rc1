# Mesh spin example - requires CGAL for DistanceMap support

if(CGAL)
  add_executable(mesh_spin mesh_spin.cpp)
  target_link_libraries(mesh_spin pe ${PE_LINK_LIBRARIES})

  # Set output directory for the executable
  set_target_properties(mesh_spin PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/mesh_spin"
  )

  message(STATUS "Configured mesh_spin example (requires CGAL)")
else()
  message(STATUS "Skipping mesh_spin example (requires CGAL)")
endif()
