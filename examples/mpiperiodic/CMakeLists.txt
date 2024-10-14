add_executable(mpiperiodic mpiperiodic.cpp)
target_include_directories(mpiperiodic PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpiperiodic PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)