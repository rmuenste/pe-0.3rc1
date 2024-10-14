add_executable(mpi3periodic mpi3periodic.cpp)

target_include_directories(mpi3periodic PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpi3periodic PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)