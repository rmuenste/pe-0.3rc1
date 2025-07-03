add_executable(convex_hull convex_hull.cpp)

# Include directories
target_include_directories(convex_hull PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})

# Link with PE library
target_link_libraries(convex_hull PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

# CGAL-specific configuration (only if CGAL is enabled)
if(CGAL)
  # Find required CGAL dependencies
  find_package(GMP REQUIRED)
  target_link_libraries(convex_hull PRIVATE ${GMP_LIBRARIES})
  
  # Add CGAL include directories if available
  if(CGAL_INCLUDE_DIR)
    target_include_directories(convex_hull PRIVATE ${CGAL_INCLUDE_DIR})
  endif()
  
  # Add CGAL dependency
  add_dependencies(convex_hull cgal)
  
  message(STATUS "convex_hull example configured with CGAL support")
else()
  message(STATUS "convex_hull example configured without CGAL support (will demonstrate fallback behavior)")
endif()