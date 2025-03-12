add_executable(cgal_box cgal_box.cpp)
target_include_directories(cgal_box PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR})
target_link_libraries(cgal_box PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

# Add CGAL dependency
add_dependencies(cgal_box cgal)