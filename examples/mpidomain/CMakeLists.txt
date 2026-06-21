add_executable(mpidomain mpidomain.cpp)
target_include_directories(mpidomain PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpidomain PRIVATE 
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)