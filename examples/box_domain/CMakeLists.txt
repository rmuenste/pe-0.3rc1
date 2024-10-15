add_executable(box_domain box_domain.cpp)
target_include_directories(box_domain PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(box_domain PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
