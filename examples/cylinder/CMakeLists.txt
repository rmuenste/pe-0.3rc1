add_executable(cylinder cylinder_test.cpp)
target_include_directories(cylinder PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(cylinder PRIVATE 
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)