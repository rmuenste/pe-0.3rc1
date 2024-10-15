add_executable(mpiboxes mpiboxes.cpp)
target_include_directories(mpiboxes PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpiboxes PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)