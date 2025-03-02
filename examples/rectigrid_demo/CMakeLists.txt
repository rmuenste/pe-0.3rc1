add_executable(rectigrid_demo rectigrid_demo.cpp)
target_include_directories(rectigrid_demo PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(rectigrid_demo PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)