add_executable(kdop_example kdop_example.cpp)
target_include_directories(kdop_example PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(kdop_example PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)