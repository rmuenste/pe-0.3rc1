add_executable(detection detection.cpp)
target_include_directories(detection PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(detection PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
