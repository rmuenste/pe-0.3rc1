add_executable(mpidem mpidem.cpp)
target_include_directories(mpidem PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpidem PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)