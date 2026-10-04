# Conveyor belt example: pe re-implementation of the LIGGGHTS-PUBLIC tutorial
# examples/LIGGGHTS/Tutorials_public/conveyor (semi-implicit hard-contact solver; serial or MPI).
add_executable(conveyor conveyor.cpp)
target_include_directories(conveyor PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../liggghts_ports)
target_link_libraries(conveyor PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)

# Self-checking runs (the example's exit code: particle count, container bounds, finite energies;
# 77 = unintended solver, reported as skipped). Serial, and under MPI with 2 and 4 processes.
# Each with the Baumgarte term and with the split-impulse position correction ("-split").
foreach(variant "" "-split")
  if(variant STREQUAL "-split")
    set(extra --split-impulse)
  else()
    set(extra)
  endif()
  add_test(NAME pe-example-conveyor-serial${variant} COMMAND conveyor --no-vtk --tend 0.7 ${extra})
  set_tests_properties(pe-example-conveyor-serial${variant} PROPERTIES
    LABELS "integration;serial;example;liggghts" SKIP_RETURN_CODE 77 TIMEOUT 900)
  if(PE_USE_MPI AND MPI_FOUND)
    foreach(np 2 4)
      add_test(NAME pe-example-conveyor-mpi${np}${variant}
        COMMAND ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} ${np} ${MPIEXEC_PREFLAGS}
                $<TARGET_FILE:conveyor> ${MPIEXEC_POSTFLAGS} --no-vtk --tend 0.7 ${extra})
      set_tests_properties(pe-example-conveyor-mpi${np}${variant} PROPERTIES
        LABELS "integration;mpi;example;liggghts" SKIP_RETURN_CODE 77 TIMEOUT 900 PROCESSORS ${np})
    endforeach()
  endif()
endforeach()
