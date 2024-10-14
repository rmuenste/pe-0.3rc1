add_executable(mpiperiodic_dev mpiperiodic_dev.cpp)
target_include_directories(mpiperiodic_dev PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpiperiodic_dev PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)