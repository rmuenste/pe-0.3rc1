add_executable(decomposition decomposition.cpp)
target_include_directories(decomposition PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(decomposition PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)