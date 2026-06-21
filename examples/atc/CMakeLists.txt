add_executable(atc_example atc_example.cpp)
target_include_directories(atc_example PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(atc_example PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)

if( PE_USE_CGAL )
  find_package(GMP REQUIRED)
  target_link_libraries(atc_example PRIVATE ${GMP_LIBRARIES})
  target_include_directories(atc_example PRIVATE ${CGAL_INCLUDE_DIR})
endif( PE_USE_CGAL )
