add_executable(sphere_column_serial_hcl sphere_column_serial_hcl.cpp)
target_include_directories(sphere_column_serial_hcl PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(sphere_column_serial_hcl PRIVATE
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
