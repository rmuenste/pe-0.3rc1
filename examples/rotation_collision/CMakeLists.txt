add_executable(rotation_collision rotation_collision.cpp)
target_include_directories(rotation_collision PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(rotation_collision PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)