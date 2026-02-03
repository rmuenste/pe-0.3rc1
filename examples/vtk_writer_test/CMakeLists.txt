# Serial version
add_executable(vtk_writer_test vtk_writer_test.cpp)
target_include_directories(vtk_writer_test PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(vtk_writer_test PRIVATE
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

# MPI parallel version
if(MPI)
  add_executable(vtk_writer_test_mpi vtk_writer_test_mpi.cpp)
  target_include_directories(vtk_writer_test_mpi PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
  target_link_libraries(vtk_writer_test_mpi PRIVATE
    $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
    $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
    $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
  )
endif()
