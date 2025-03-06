add_executable(body_removal body_removal.cpp)
target_include_directories(body_removal PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(body_removal PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)