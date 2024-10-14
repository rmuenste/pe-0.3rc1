add_executable(mpibench mpibench.cpp)
target_include_directories(mpibench PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpibench PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)