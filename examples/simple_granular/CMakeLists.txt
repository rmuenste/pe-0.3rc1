add_executable(simple_granular simple_granular.cpp)
target_include_directories(simple_granular PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(simple_granular PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)